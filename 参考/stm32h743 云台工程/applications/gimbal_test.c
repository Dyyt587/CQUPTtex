#include "gimbal_interpolation.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include "finsh.h"

// 测试相关的宏定义
#define GIMBAL_TEST_FEED_RATE    30.0f    // 测试进给速度 30度/秒
#define GIMBAL_INTERP_PERIOD     50       // 插补周期 50ms

// 外部变量声明 - 来自drv_emm_v5.c
extern stepper_motor_t stepper_motor_1;  // X轴电机
extern stepper_motor_t stepper_motor_2;  // Y轴电机

// 测试回调函数
static void test_step_callback(point_t current, point_t target)
{
    rt_kprintf("Step: Current(%.2f, %.2f) -> Target(%.2f, %.2f)\n", 
               current.x, current.y, target.x, target.y);
}

static void test_finish_callback(void)
{
    rt_kprintf("=== Motion Finished ===\n");
}

/**
 * @brief 初始化云台插补系统
 */
static int gimbal_interp_init_test(void)
{
    // 初始化插补系统
    int ret = gimbal_interpolation_init(&gimbal_interp, 
                                       &stepper_motor_1,  // X轴电机
                                       &stepper_motor_2,  // Y轴电机  
                                       GIMBAL_INTERP_PERIOD);
    if (ret != 0) {
        rt_kprintf("Failed to initialize gimbal interpolation\n");
        return ret;
    }
    
    // 设置回调函数
    gimbal_set_step_callback(&gimbal_interp, test_step_callback);
    gimbal_set_finish_callback(&gimbal_interp, test_finish_callback);
    
    // 启动插补定时器
    ret = gimbal_start_interpolation_timer();
    if (ret != 0) {
        rt_kprintf("Failed to start interpolation timer\n");
        return ret;
    }
    
    rt_kprintf("Gimbal interpolation system initialized successfully\n");
    return 0;
}

/**
 * @brief 测试直线插补 - 正方形轨迹
 */
static void test_square_trajectory(void)
{
    rt_kprintf("\n=== Testing Square Trajectory ===\n");
    
    // 定义正方形的四个顶点 (以度为单位)
    point_t square_points[] = {
        {0.0f, 0.0f},      // 起点
        {30.0f, 0.0f},     // 第一条边
        {30.0f, 30.0f},    // 第二条边
        {0.0f, 30.0f},     // 第三条边
        {0.0f, 0.0f}       // 回到起点
    };
    
    int num_points = sizeof(square_points) / sizeof(point_t);
    
    for (int i = 1; i < num_points; i++) {
        rt_kprintf("Moving to point %d: (%.1f, %.1f)\n", 
                   i, square_points[i].x, square_points[i].y);
        
        // 设置直线插补
        int ret = gimbal_set_line_interpolation(&gimbal_interp,
                                               square_points[i-1].x, square_points[i-1].y,
                                               square_points[i].x, square_points[i].y,
                                               GIMBAL_TEST_FEED_RATE);
        if (ret != 0) {
            rt_kprintf("Failed to set line interpolation\n");
            return;
        }
        
        // 开始运动
        ret = gimbal_start_interpolation(&gimbal_interp);
        if (ret != 0) {
            rt_kprintf("Failed to start interpolation\n");
            return;
        }
        
        // 等待运动完成
        ret = gimbal_wait_motion_finish(10000); // 10秒超时
        if (ret != 0) {
            rt_kprintf("Motion timeout or error\n");
            return;
        }
        
        rt_thread_mdelay(500); // 暂停500ms
    }
    
    rt_kprintf("Square trajectory test completed!\n");
}

/**
 * @brief 测试对角线运动
 */
static void test_diagonal_motion(void)
{
    rt_kprintf("\n=== Testing Diagonal Motion ===\n");
    
    // 从(0,0)移动到(45,45)
    int ret = gimbal_g01_line(45.0f, 45.0f, GIMBAL_TEST_FEED_RATE);
    if (ret != 0) {
        rt_kprintf("Failed to start diagonal motion\n");
        return;
    }
    
    // 等待完成
    ret = gimbal_wait_motion_finish(5000);
    if (ret == 0) {
        rt_kprintf("Diagonal motion completed successfully\n");
    } else {
        rt_kprintf("Diagonal motion timeout\n");
    }
}

/**
 * @brief 测试回零功能
 */
static void test_home_position(void)
{
    rt_kprintf("\n=== Moving to Home Position ===\n");
    
    // 移动到原点
    int ret = gimbal_g01_line(0.0f, 0.0f, GIMBAL_TEST_FEED_RATE);
    if (ret != 0) {
        rt_kprintf("Failed to move to home position\n");
        return;
    }
    
    ret = gimbal_wait_motion_finish(5000);
    if (ret == 0) {
        rt_kprintf("Successfully returned to home position\n");
    } else {
        rt_kprintf("Home position timeout\n");
    }
}

/**
 * @brief 获取当前位置信息
 */
static void show_current_position(void)
{
    point_t pos;
    gimbal_get_current_position(&gimbal_interp, &pos);
    
    interpolation_state_t state = gimbal_get_interpolation_state(&gimbal_interp);
    const char* state_str[] = {"IDLE", "RUNNING", "FINISHED", "ERROR"};
    
    rt_kprintf("Current Position: (%.3f, %.3f)\n", pos.x, pos.y);
    rt_kprintf("Motor Angles: X=%.3f, Y=%.3f\n", 
               stepper_motor_1.stepper_motor_angle,
               stepper_motor_2.stepper_motor_angle);
    rt_kprintf("State: %s\n", state_str[state]);
}

/**
 * @brief 停止当前运动
 */
static void stop_motion(void)
{
    int ret = gimbal_stop_interpolation(&gimbal_interp);
    if (ret == 0) {
        rt_kprintf("Motion stopped\n");
    } else {
        rt_kprintf("Failed to stop motion\n");
    }
}

// MSH命令行接口函数
static int gimbal_init(int argc, char **argv)
{
    return gimbal_interp_init_test();
}
MSH_CMD_EXPORT(gimbal_init, Initialize gimbal interpolation system);

static void gimbal_square(int argc, char **argv)
{
    test_square_trajectory();
}
MSH_CMD_EXPORT(gimbal_square, Test square trajectory);

static void gimbal_diagonal(int argc, char **argv)
{
    test_diagonal_motion();
}
MSH_CMD_EXPORT(gimbal_diagonal, Test diagonal motion);

static void gimbal_home(int argc, char **argv)
{
    test_home_position();
}
MSH_CMD_EXPORT(gimbal_home, Move to home position);

static void gimbal_pos(int argc, char **argv)
{
    show_current_position();
}
MSH_CMD_EXPORT(gimbal_pos, Show current position);

static void gimbal_stop(int argc, char **argv)
{
    stop_motion();
}
MSH_CMD_EXPORT(gimbal_stop, Stop current motion);

static void gimbal_move(int argc, char **argv)
{
    if (argc != 3 && argc != 4) {
        rt_kprintf("Usage: gimbal_move <x> <y> [feed_rate]\n");
        rt_kprintf("Example: gimbal_move 30 45 25\n");
        return;
    }
    
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    float feed_rate = (argc == 4) ? atof(argv[3]) : GIMBAL_TEST_FEED_RATE;
    
    rt_kprintf("Moving to (%.2f, %.2f) at %.2f deg/s\n", x, y, feed_rate);
    
    int ret = gimbal_g01_line(x, y, feed_rate);
    if (ret != 0) {
        rt_kprintf("Failed to start motion\n");
        return;
    }
    
    rt_kprintf("Motion started, use 'gimbal_pos' to check status\n");
}
MSH_CMD_EXPORT(gimbal_move, Move to specified position);

/**
 * @brief 云台插补测试线程
 */
static void gimbal_test_thread(void *parameter)
{
    rt_kprintf("Gimbal Interpolation Test Thread Started\n");
    rt_kprintf("Available commands:\n");
    rt_kprintf("  gimbal_init     - Initialize system\n");
    rt_kprintf("  gimbal_move x y [speed] - Move to position\n");
    rt_kprintf("  gimbal_square   - Test square trajectory\n");
    rt_kprintf("  gimbal_diagonal - Test diagonal motion\n");
    rt_kprintf("  gimbal_home     - Move to home position\n");
    rt_kprintf("  gimbal_pos      - Show current position\n");
    rt_kprintf("  gimbal_stop     - Stop current motion\n");
    rt_kprintf("\nExample usage:\n");
    rt_kprintf("  gimbal_init\n");
    rt_kprintf("  gimbal_move 30 45 25\n");
    rt_kprintf("  gimbal_square\n");
    
    // 自动初始化系统
    rt_thread_mdelay(1000);
    gimbal_interp_init_test();
    
    while (1) {
        rt_thread_mdelay(1000);
    }
}

/**
 * @brief 启动云台插补测试
 */
static int gimbal_test_start(void)
{
    rt_thread_t tid = rt_thread_create("gimbal_test",
                                      gimbal_test_thread,
                                      RT_NULL,
                                      2048,
                                      20,
                                      20);
    if (tid == RT_NULL) {
        rt_kprintf("Failed to create gimbal test thread\n");
        return -1;
    }
    
    rt_thread_startup(tid);
    rt_kprintf("Gimbal test thread started\n");
    return 0;
}
MSH_CMD_EXPORT(gimbal_test_start, Start gimbal interpolation test);

// 自动初始化
INIT_APP_EXPORT(gimbal_test_start);
