/**
 * @file gimbal_simple_test.c
 * @brief 简化版云台插补测试 - 修复中断互斥锁问题
 * @version 1.1
 * @date 2025-07-25
 */

#include "gimbal_interpolation.h"
#include "drv_emm_v5.h"
#include <rtthread.h>

#define GIMBAL_TEST_FEED_RATE    20.0f    // 测试进给速度 20度/秒
#define GIMBAL_INTERP_PERIOD     50       // 插补周期 50ms

// 外部变量声明
extern stepper_motor_t stepper_motor_1;  // X轴电机
extern stepper_motor_t stepper_motor_2;  // Y轴电机

// 测试状态标志
static volatile bool test_running = false;

/**
 * @brief 简单的步进回调函数
 */
static void simple_step_callback(point_t current, point_t target)
{
    // 简化输出，避免在中断中使用复杂的日志功能
    // LOG_D("Step: (%.1f,%.1f) -> (%.1f,%.1f)", current.x, current.y, target.x, target.y);
}

/**
 * @brief 完成回调函数
 */
static void simple_finish_callback(void)
{
    test_running = false;
    rt_kprintf("Motion completed!\n");
}

/**
 * @brief 安全的初始化函数
 */
static int safe_gimbal_init(void)
{
    rt_kprintf("Initializing gimbal interpolation system...\n");
    
    // 初始化插补系统
    int ret = gimbal_interpolation_init(&gimbal_interp, 
                                       &stepper_motor_1,  // X轴电机
                                       &stepper_motor_2,  // Y轴电机  
                                       GIMBAL_INTERP_PERIOD);
    if (ret != 0) {
        rt_kprintf("Failed to initialize gimbal interpolation: %d\n", ret);
        return ret;
    }
    
    // 设置回调函数
    gimbal_set_step_callback(&gimbal_interp, simple_step_callback);
    gimbal_set_finish_callback(&gimbal_interp, simple_finish_callback);
    
    // 启动插补定时器
    ret = gimbal_start_interpolation_timer();
    if (ret != 0) {
        rt_kprintf("Failed to start interpolation timer: %d\n", ret);
        gimbal_interpolation_deinit(&gimbal_interp);
        return ret;
    }
    
    rt_kprintf("Gimbal interpolation system initialized successfully!\n");
    rt_kprintf("System uses message queue to avoid mutex issues in ISR.\n");
    return 0;
}

/**
 * @brief 安全的移动测试
 */
static void safe_move_test(float x, float y, float speed)
{
    rt_kprintf("Moving to (%.1f, %.1f) at %.1f deg/s...\n", x, y, speed);
    
    test_running = true;
    
    int ret = gimbal_g01_line(x, y, speed);
    if (ret != 0) {
        rt_kprintf("Failed to start motion: %d\n", ret);
        test_running = false;
        return;
    }
    
    // 等待完成
    uint32_t timeout = 0;
    while (test_running && timeout < 100) {  // 10秒超时
        rt_thread_mdelay(100);
        timeout++;
    }
    
    if (timeout >= 100) {
        rt_kprintf("Motion timeout!\n");
    }
}

/**
 * @brief 简单的正方形轨迹测试
 */
static void simple_square_test(void)
{
    rt_kprintf("\n=== Simple Square Trajectory Test ===\n");
    
    // 正方形的四个点
    point_t points[] = {
        {10.0f, 10.0f},    // 第一个点
        {30.0f, 10.0f},    // 第二个点
        {30.0f, 30.0f},    // 第三个点
        {10.0f, 30.0f},    // 第四个点
        {10.0f, 10.0f}     // 回到起点
    };
    
    for (int i = 0; i < 5; i++) {
        safe_move_test(points[i].x, points[i].y, GIMBAL_TEST_FEED_RATE);
        rt_thread_mdelay(500);  // 每次移动后暂停
    }
    
    rt_kprintf("Square trajectory test completed!\n");
}

/**
 * @brief 显示当前位置 - 线程安全版本
 */
static void show_position_safe(void)
{
    point_t pos;
    gimbal_get_current_position(&gimbal_interp, &pos);
    
    interpolation_state_t state = gimbal_get_interpolation_state(&gimbal_interp);
    const char* state_names[] = {"IDLE", "RUNNING", "FINISHED", "ERROR"};
    
    rt_kprintf("Position: (%.2f, %.2f), State: %s\n", 
               pos.x, pos.y, state_names[state]);
}

// 控制台命令导出
#include "finsh.h"

static void gimbal_safe_init(int argc, char **argv)
{
    safe_gimbal_init();
}
MSH_CMD_EXPORT(gimbal_safe_init, Safe initialize gimbal system);

static void gimbal_safe_move(int argc, char **argv)
{
    if (argc != 3 && argc != 4) {
        rt_kprintf("Usage: gimbal_safe_move <x> <y> [speed]\n");
        return;
    }
    
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    float speed = (argc == 4) ? atof(argv[3]) : GIMBAL_TEST_FEED_RATE;
    
    safe_move_test(x, y, speed);
}
MSH_CMD_EXPORT(gimbal_safe_move, Safe move to position);

static void gimbal_safe_square(int argc, char **argv)
{
    simple_square_test();
}
MSH_CMD_EXPORT(gimbal_safe_square, Safe square trajectory test);

static void gimbal_safe_pos(int argc, char **argv)
{
    show_position_safe();
}
MSH_CMD_EXPORT(gimbal_safe_pos, Show current position safely);

static void gimbal_safe_home(int argc, char **argv)
{
    safe_move_test(0.0f, 0.0f, GIMBAL_TEST_FEED_RATE);
}
MSH_CMD_EXPORT(gimbal_safe_home, Safe move to home position);

/**
 * @brief 系统状态检查
 */
static void gimbal_status(int argc, char **argv)
{
    rt_kprintf("\n=== Gimbal System Status ===\n");
    rt_kprintf("Version: 1.1 (ISR-Safe)\n");
    rt_kprintf("Features: Message queue based motor control\n");
    rt_kprintf("Period: %d ms\n", gimbal_interp.interpolation_period_ms);
    rt_kprintf("Queue: %s\n", gimbal_interp.motor_cmd_queue ? "OK" : "ERROR");
    
    show_position_safe();
    
    rt_kprintf("\nAvailable commands:\n");
    rt_kprintf("  gimbal_safe_init  - Initialize system\n");
    rt_kprintf("  gimbal_safe_move  - Move to position\n");
    rt_kprintf("  gimbal_safe_square - Test square path\n");
    rt_kprintf("  gimbal_safe_pos   - Show position\n");
    rt_kprintf("  gimbal_safe_home  - Go to origin\n");
    rt_kprintf("=============================\n");
}
MSH_CMD_EXPORT(gimbal_status, Show gimbal system status);

/**
 * @brief 自动初始化线程
 */
static void gimbal_auto_init_thread(void *parameter)
{
    rt_thread_mdelay(2000);  // 等待系统启动完成
    
    rt_kprintf("\n=== Gimbal Interpolation System ===\n");
    rt_kprintf("Auto-initializing in 1 second...\n");
    rt_kprintf("Fixed: ISR mutex issue using message queue\n");
    
    rt_thread_mdelay(1000);
    
    int ret = safe_gimbal_init();
    if (ret == 0) {
        rt_kprintf("System ready! Use 'gimbal_status' for help.\n");
    } else {
        rt_kprintf("Auto-init failed, use 'gimbal_safe_init' manually.\n");
    }
}

/**
 * @brief 启动自动初始化
 */
static int gimbal_auto_start(void)
{
    rt_thread_t tid = rt_thread_create("gimbal_auto",
                                      gimbal_auto_init_thread,
                                      RT_NULL,
                                      1024,
                                      25,
                                      10);
    if (tid != RT_NULL) {
        rt_thread_startup(tid);
        return 0;
    }
    return -1;
}

// 系统启动时自动初始化
INIT_APP_EXPORT(gimbal_auto_start);
