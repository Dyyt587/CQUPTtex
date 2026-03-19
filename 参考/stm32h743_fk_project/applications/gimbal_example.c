/**
 * @file gimbal_example.c
 * @brief 二维云台直线插补简单使用示例
 * @version 1.0
 * @date 2025-07-25
 */

#include "gimbal_interpolation.h"
#include "drv_emm_v5.h"
#include <rtthread.h>

/**
 * @brief 简单的直线插补示例
 * 
 * 这个示例展示了如何使用直线插补功能让云台沿直线运动
 */
void gimbal_simple_example(void)
{
    // 1. 初始化插补系统
    int ret = gimbal_interpolation_init(&gimbal_interp, 
                                       &stepper_motor_1,  // X轴电机
                                       &stepper_motor_2,  // Y轴电机
                                       50);               // 50ms插补周期
    if (ret != 0) {
        rt_kprintf("Failed to initialize gimbal interpolation\n");
        return;
    }
    
    // 2. 启动插补定时器
    ret = gimbal_start_interpolation_timer();
    if (ret != 0) {
        rt_kprintf("Failed to start interpolation timer\n");
        return;
    }
    
    rt_kprintf("Gimbal interpolation initialized successfully\n");
    
    // 3. 执行一系列直线运动
    rt_kprintf("Starting line interpolation demo...\n");
    
    // 移动到(30, 0)
    rt_kprintf("Moving to (30, 0)...\n");
    gimbal_g01_line(30.0f, 0.0f, 20.0f);    // 20度/秒
    gimbal_wait_motion_finish(5000);         // 等待5秒
    
    // 移动到(30, 30)  
    rt_kprintf("Moving to (30, 30)...\n");
    gimbal_g01_line(30.0f, 30.0f, 20.0f);
    gimbal_wait_motion_finish(5000);
    
    // 移动到(0, 30)
    rt_kprintf("Moving to (0, 30)...\n");
    gimbal_g01_line(0.0f, 30.0f, 20.0f);
    gimbal_wait_motion_finish(5000);
    
    // 回到原点
    rt_kprintf("Moving back to origin...\n");
    gimbal_g01_line(0.0f, 0.0f, 20.0f);
    gimbal_wait_motion_finish(5000);
    
    rt_kprintf("Line interpolation demo completed!\n");
}

/**
 * @brief 高精度直线插补示例
 * 
 * 使用更小的步长和更高的精度进行插补
 */
void gimbal_precision_example(void)
{
    rt_kprintf("Starting precision interpolation demo...\n");
    
    // 使用更小的插补周期和更慢的速度以获得更高精度
    gimbal_interp.interpolation_period_ms = 20;  // 20ms周期
    
    // 执行精密的对角线运动
    rt_kprintf("Precision diagonal movement...\n");
    gimbal_g01_line(45.0f, 45.0f, 10.0f);   // 10度/秒慢速
    gimbal_wait_motion_finish(0);            // 无限等待直到完成
    
    rt_kprintf("Precision demo completed!\n");
}

/**
 * @brief 带回调函数的插补示例
 */
static void my_step_callback(point_t current, point_t target)
{
    rt_kprintf("Step: (%.2f, %.2f) -> (%.2f, %.2f)\n", 
               current.x, current.y, target.x, target.y);
}

static void my_finish_callback(void)
{
    rt_kprintf("Motion finished!\n");
}

void gimbal_callback_example(void)
{
    rt_kprintf("Starting callback demo...\n");
    
    // 设置回调函数
    gimbal_set_step_callback(&gimbal_interp, my_step_callback);
    gimbal_set_finish_callback(&gimbal_interp, my_finish_callback);
    
    // 执行运动 - 每一步都会调用回调函数
    gimbal_g01_line(20.0f, 20.0f, 15.0f);
    gimbal_wait_motion_finish(0);
}

/**
 * @brief 连续轨迹示例 - 绘制字母"L"
 */
void gimbal_draw_letter_L(void)
{
    rt_kprintf("Drawing letter 'L'...\n");
    
    // 移动到起始位置(不绘制)
    gimbal_g01_line(10.0f, 40.0f, 30.0f);
    gimbal_wait_motion_finish(0);
    
    // 绘制竖线
    rt_kprintf("Drawing vertical line...\n");
    gimbal_g01_line(10.0f, 10.0f, 15.0f);
    gimbal_wait_motion_finish(0);
    
    // 绘制横线
    rt_kprintf("Drawing horizontal line...\n");
    gimbal_g01_line(30.0f, 10.0f, 15.0f);
    gimbal_wait_motion_finish(0);
    
    rt_kprintf("Letter 'L' completed!\n");
}

/**
 * @brief 状态监控示例
 */
void gimbal_monitor_example(void)
{
    rt_kprintf("Starting motion with monitoring...\n");
    
    // 开始一个较长的运动
    gimbal_g01_line(50.0f, 50.0f, 10.0f);
    
    // 监控运动状态
    while (!gimbal_is_motion_finished(&gimbal_interp)) {
        point_t pos;
        gimbal_get_current_position(&gimbal_interp, &pos);
        
        interpolation_state_t state = gimbal_get_interpolation_state(&gimbal_interp);
        rt_kprintf("Current: (%.2f, %.2f), State: %d\n", pos.x, pos.y, state);
        
        rt_thread_mdelay(500);  // 每500ms检查一次
    }
    
    rt_kprintf("Motion monitoring completed!\n");
}

/**
 * @brief 主要的示例函数，包含多个测试
 */
void gimbal_interpolation_examples(void)
{
    rt_kprintf("\n=== Gimbal Interpolation Examples ===\n");
    
    // 确保电机已初始化
    rt_thread_mdelay(2000);
    
    // 运行各种示例
    gimbal_simple_example();      // 基础示例
    rt_thread_mdelay(1000);
    
    gimbal_precision_example();   // 精度示例  
    rt_thread_mdelay(1000);
    
    gimbal_callback_example();    // 回调示例
    rt_thread_mdelay(1000);
    
    gimbal_draw_letter_L();       // 绘制示例
    rt_thread_mdelay(1000);
    
    gimbal_monitor_example();     // 监控示例
    
    rt_kprintf("\n=== All Examples Completed ===\n");
}

// 导出到控制台命令
#include "finsh.h"
MSH_CMD_EXPORT(gimbal_simple_example, Run simple gimbal interpolation example);
MSH_CMD_EXPORT(gimbal_precision_example, Run precision interpolation example);
MSH_CMD_EXPORT(gimbal_callback_example, Run callback example);
MSH_CMD_EXPORT(gimbal_draw_letter_L, Draw letter L with gimbal);
MSH_CMD_EXPORT(gimbal_monitor_example, Run monitoring example);
MSH_CMD_EXPORT(gimbal_interpolation_examples, Run all gimbal examples);
