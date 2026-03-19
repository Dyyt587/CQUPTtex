/**
 * @file gimbal_direct_test.c
 * @brief 直接在定时器中控制电机的云台插补测试 - 解决队列延时问题
 * @version 1.2
 * @date 2025-07-25
 */

#include "gimbal_interpolation.h"
#include "drv_emm_v5.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <rtthread.h>
#include "ulog.h"

#define GIMBAL_TEST_FEED_RATE    30.0f    // 测试进给速度 30度/秒
#define GIMBAL_INTERP_PERIOD     50       // 插补周期 50ms

// 外部变量声明
extern stepper_motor_t stepper_motor_1;  // X轴电机
extern stepper_motor_t stepper_motor_2;  // Y轴电机

// 测试状态标志
static volatile bool test_running = false;

/**
 * @brief 简单的步进回调函数
 */
static void direct_step_callback(point_t current, point_t target)
{
    // 在中断中避免复杂的输出
    static int step_counter = 0;
    step_counter++;
    if (step_counter % 10 == 0) {  // 每10步输出一次
        // 使用简单的输出，避免格式化
        // LOG_D("Step %d\n", step_counter);
    }
}

/**
 * @brief 完成回调函数
 */
static void direct_finish_callback(void)
{
    test_running = false;
    LOG_D("=== Motion completed! ===\n");
}

/**
 * @brief 直接控制版本的初始化函数
 */
static int direct_gimbal_init(void)
{
    LOG_D("Initializing direct gimbal interpolation system...\n");
    LOG_D("Version: 1.2 - Direct timer control (no queue delay)\n");
    
    // 初始化插补系统
    int ret = gimbal_interpolation_init(&gimbal_interp, 
                                       &stepper_motor_1,  // X轴电机
                                       &stepper_motor_2,  // Y轴电机  
                                       GIMBAL_INTERP_PERIOD);
    if (ret != 0) {
        LOG_D("Failed to initialize gimbal interpolation: %d\n", ret);
        return ret;
    }
    
    // 设置回调函数
    gimbal_set_step_callback(&gimbal_interp, direct_step_callback);
    gimbal_set_finish_callback(&gimbal_interp, direct_finish_callback);
    
    // 启动插补定时器
    ret = gimbal_start_interpolation_timer();
    if (ret != 0) {
        LOG_D("Failed to start interpolation timer: %d\n", ret);
        gimbal_interpolation_deinit(&gimbal_interp);
        return ret;
    }
    
    LOG_D("Direct gimbal interpolation initialized successfully!\n");
    LOG_D("Features: Real-time motor control in timer ISR\n");
    LOG_D("Benefits: No queue delay, immediate response\n");
    return 0;
}

/**
 * @brief 直接控制版本的移动测试
 */
static void direct_move_test(float x, float y, float speed)
{
    LOG_D("Direct move to (%.1f, %.1f) at %.1f deg/s...\n", x, y, speed);
    
    test_running = true;
    
    int ret = gimbal_g01_line(x, y, speed);
    if (ret != 0) {
        LOG_D("Failed to start motion: %d\n", ret);
        test_running = false;
        return;
    }
    
    // 等待完成
    uint32_t timeout = 0;
    while (test_running && timeout < 150) {  // 15秒超时
        rt_thread_mdelay(100);
        timeout++;
        
        // 每2秒显示一次当前位置
        if (timeout % 20 == 0) {
            point_t pos;
            gimbal_get_current_position(&gimbal_interp, &pos);
            LOG_D("Current: (%.2f, %.2f)\n", pos.x, pos.y);
        }
    }
    
    if (timeout >= 150) {
        LOG_D("Motion timeout!\n");
        gimbal_stop_interpolation(&gimbal_interp);
    }
}

/**
 * @brief 直接控制版本的正方形轨迹测试
 */
static void direct_square_test(void)
{
    LOG_D("\n=== Direct Square Trajectory Test ===\n");
    
    // 正方形的四个点
    point_t points[] = {
        {10.0f, 10.0f},    // 第一个点
        {40.0f, 10.0f},    // 第二个点
        {40.0f, 40.0f},    // 第三个点
        {10.0f, 40.0f},    // 第四个点
        {10.0f, 10.0f}     // 回到起点
    };
    
    for (int i = 0; i < 5; i++) {
        LOG_D("Segment %d: Moving to (%.1f, %.1f)\n", 
                   i+1, points[i].x, points[i].y);
        direct_move_test(points[i].x, points[i].y, GIMBAL_TEST_FEED_RATE);
        rt_thread_mdelay(1000);  // 每次移动后暂停1秒
    }
    
    LOG_D("Direct square trajectory test completed!\n");
}

/**
 * @brief 性能测试 - 连续小步移动
 */
static void direct_performance_test(void)
{
    LOG_D("\n=== Performance Test - Continuous Small Steps ===\n");
    
    float start_x = 0.0f, start_y = 0.0f;
    float step_size = 5.0f;
    int steps = 8;
    
    for (int i = 0; i < steps; i++) {
        float target_x = start_x + step_size * (i % 2);
        float target_y = start_y + step_size * (i / 2);
        
        LOG_D("Step %d: (%.1f, %.1f)\n", i+1, target_x, target_y);
        direct_move_test(target_x, target_y, 40.0f);  // 快速移动
        rt_thread_mdelay(200);  // 短暂停留
    }
    
    LOG_D("Performance test completed!\n");
}

/**
 * @brief 显示当前位置和系统状态
 */
static void direct_show_status(void)
{
    point_t pos;
    gimbal_get_current_position(&gimbal_interp, &pos);
    
    interpolation_state_t state = gimbal_get_interpolation_state(&gimbal_interp);
    const char* state_names[] = {"IDLE", "RUNNING", "FINISHED", "ERROR"};
    
    LOG_D("\n=== Direct Control System Status ===\n");
    LOG_D("Version: 1.2 (Timer Direct Control)\n");
    LOG_D("Position: (%.3f, %.3f)\n", pos.x, pos.y);
    LOG_D("State: %s\n", state_names[state]);
    LOG_D("Period: %d ms\n", gimbal_interp.interpolation_period_ms);
    LOG_D("Running: %s\n", test_running ? "YES" : "NO");
    
    // 显示电机状态
    LOG_D("Motor X: ID=%d, Angle=%.2f, Target=%.2f\n", 
               stepper_motor_1.stepper_motor_id,
               stepper_motor_1.stepper_motor_angle,
               stepper_motor_1.stepper_motor_target_angle);
    LOG_D("Motor Y: ID=%d, Angle=%.2f, Target=%.2f\n", 
               stepper_motor_2.stepper_motor_id,
               stepper_motor_2.stepper_motor_angle,
               stepper_motor_2.stepper_motor_target_angle);
    LOG_D("=====================================\n");
}

// 控制台命令导出
#include "finsh.h"

static void gimbal_direct_init(int argc, char **argv)
{
    direct_gimbal_init();
}
MSH_CMD_EXPORT(gimbal_direct_init, Initialize direct gimbal system);

static void gimbal_direct_move(int argc, char **argv)
{
    if (argc != 3 && argc != 4) {
        LOG_D("Usage: gimbal_direct_move <x> <y> [speed]\n");
        return;
    }
    
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    float speed = (argc == 4) ? atof(argv[3]) : GIMBAL_TEST_FEED_RATE;
    
    direct_move_test(x, y, speed);
}
MSH_CMD_EXPORT(gimbal_direct_move, Direct move to position);

static void gimbal_direct_square(int argc, char **argv)
{
    direct_square_test();
}
MSH_CMD_EXPORT(gimbal_direct_square, Direct square trajectory test);

static void gimbal_direct_perf(int argc, char **argv)
{
    direct_performance_test();
}
MSH_CMD_EXPORT(gimbal_direct_perf, Performance test with small steps);

static void gimbal_direct_status(int argc, char **argv)
{
    direct_show_status();
}
MSH_CMD_EXPORT(gimbal_direct_status, Show direct system status);

static void gimbal_direct_home(int argc, char **argv)
{
    direct_move_test(0.0f, 0.0f, GIMBAL_TEST_FEED_RATE);
}
MSH_CMD_EXPORT(gimbal_direct_home, Direct move to home position);

static void gimbal_direct_stop(int argc, char **argv)
{
    gimbal_stop_interpolation(&gimbal_interp);
    test_running = false;
    LOG_D("Direct motion stopped\n");
}
MSH_CMD_EXPORT(gimbal_direct_stop, Stop direct motion);

/**
 * @brief 自动初始化线程
 */
static void gimbal_direct_auto_init_thread(void *parameter)
{
    rt_thread_mdelay(3000);  // 等待系统启动完成
    
    LOG_D("\n=== Direct Gimbal Interpolation System ===\n");
    LOG_D("Version 1.2 - Timer Direct Control\n");
    LOG_D("Fixed: Queue delay issue by direct ISR control\n");
    LOG_D("Auto-initializing...\n");
    
    int ret = direct_gimbal_init();
    if (ret == 0) {
        LOG_D("System ready! Commands available:\n");
        LOG_D("  gimbal_direct_move x y [speed]\n");
        LOG_D("  gimbal_direct_square\n");
        LOG_D("  gimbal_direct_perf\n");
        LOG_D("  gimbal_direct_status\n");
        LOG_D("  gimbal_direct_home\n");
        LOG_D("  gimbal_direct_stop\n");
    } else {
        LOG_D("Auto-init failed, use 'gimbal_direct_init' manually.\n");
    }
}

/**
 * @brief 启动自动初始化
 */
static int gimbal_direct_auto_start(void)
{
    rt_thread_t tid = rt_thread_create("gimbal_direct",
                                      gimbal_direct_auto_init_thread,
                                      RT_NULL,
                                      1536,
                                      25,
                                      10);
    if (tid != RT_NULL) {
        rt_thread_startup(tid);
        return 0;
    }
    return -1;
}

// 系统启动时自动初始化
INIT_APP_EXPORT(gimbal_direct_auto_start);
