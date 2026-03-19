/**
 * @file laser_gimbal_test.c
 * @brief 激光云台测试和使用示例
 * @version 1.0
 * @date 2025-07-28
 * @author Your Name
 * @copyright Copyright (c) 2025
 */

#include "laser_gimbal.h"
#include "drv_ms4010.h"
#include <rtthread.h>

#define DBG_TAG "laser_gimbal_test"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


/* 全局变量 */
 laser_gimbal_t g_laser_gimbal;
 ms4010_manager_t g_motor_manager;
 ms4010_device_t *g_x_motor = RT_NULL;      // X轴电机 (ID=1, 对应偏航轴)
 ms4010_device_t *g_y_motor = RT_NULL;      // Y轴电机 (ID=2, 对应俯仰轴)

/* 回调函数 */
static void on_target_reached(laser_coordinate_t target)
{
    //LOG_I("Target reached: (%.2f, %.2f, %.2f)", target.x, target.y, target.z);
}

static void on_gimbal_error(int error_code)
{
    LOG_E("Gimbal error occurred: code %d", error_code);
}

/**
 * @brief 初始化激光云台系统
 */
rt_err_t laser_gimbal_system_init(void)
{
    rt_err_t result;

    LOG_I("=== Initializing Laser Gimbal System ===");

    /* 1. 初始化MS4010电机管理器 */
    result = ms4010_manager_init(&g_motor_manager, "uart2", 1000000);
    if (result != RT_EOK)
    {
        LOG_E("Failed to initialize motor manager: %d", result);
        return result;
    }

    /* 2. 自动扫描并添加电机 */
    LOG_I("Scanning for motors...");
    rt_uint8_t found_count = ms4010_manager_quick_scan(&g_motor_manager);
    if (found_count < 2)
    {
        LOG_E("Need at least 2 motors, found %d", found_count);
        ms4010_manager_deinit(&g_motor_manager);
        return -RT_ERROR;
    }

    /* 3. 获取电机设备 (1号电机=X轴, 2号电机=Y轴) */
    g_x_motor = ms4010_manager_get_motor(&g_motor_manager, 1);    // X轴电机 (偏航轴)
    g_y_motor = ms4010_manager_get_motor(&g_motor_manager, 2);    // Y轴电机 (俯仰轴)

    if (g_x_motor == RT_NULL || g_y_motor == RT_NULL)
    {
        LOG_E("Failed to get motor devices (ID 1,2 required)");
        ms4010_manager_deinit(&g_motor_manager);
        return -RT_ERROR;
    }

    /* 4. 初始化激光云台系统 */
    result = laser_gimbal_init(&g_laser_gimbal, g_x_motor, g_y_motor, 0.2f); // 0.2米高度
    if (result != RT_EOK)
    {
        LOG_E("Failed to initialize laser gimbal: %d", result);
        ms4010_manager_deinit(&g_motor_manager);
        return result;
    }

    /* 5. 设置机械参数 (根据你的实际结构调整) */
    laser_gimbal_set_mechanical_params(&g_laser_gimbal, 
                                      109.33f,    // X轴(偏航)零点偏移
                                      20.76f-2.9f+0.01f,    // Y轴(俯仰)零点偏移  
                                      false,   // X轴不反向
                                      false);  // Y轴不反向

    /* 6. 设置运动参数 */
    laser_gimbal_set_motion_params(&g_laser_gimbal,
                                  720.0f,   // 最大X轴速度 45°/s
                                  720.0f,   // 最大Y轴速度 30°/s 
                                  800.0f,   // 加速度 80°/s²
                                  0.8f);   // 平滑因子

    /* 7. 设置角度限制 */
    laser_gimbal_set_angle_limits(&g_laser_gimbal,
                                 -720.0f, 360.0f,   // X轴角度范围
                                 -180.0f, 180.0f);  // Y轴角度范围

    /* 8. 设置回调函数 */
    g_laser_gimbal.on_target_reached = on_target_reached;
    g_laser_gimbal.on_error = on_gimbal_error;

    /* 9. 启用电机 */
    result = laser_gimbal_enable(&g_laser_gimbal, true);
    if (result != RT_EOK)
    {
        LOG_E("Failed to enable gimbal motors: %d", result);
        return result;
    }

    /* 10. 回零 */
    result = laser_gimbal_home(&g_laser_gimbal);
    if (result != RT_EOK)
    {
        LOG_E("Failed to home gimbal: %d", result);
        return result;
    }

    LOG_I("=== Laser Gimbal System Initialized Successfully ===");
    return RT_EOK;
}

/**
 * @brief 基本功能测试
 */
static void laser_gimbal_basic_test(void)
{
    rt_err_t result;
    laser_gimbal_angle_t current_angle;

    LOG_I("=== Starting Basic Function Test ===");

    /* 1. 测试直接角度控制 */
    LOG_I("Test 1: Direct angle control");
    result = laser_gimbal_set_angle(&g_laser_gimbal, 30.0f, 15.0f);
    if (result == RT_EOK)
    {
        rt_thread_mdelay(2000); // 等待运动完成
        laser_gimbal_get_current_angle(&g_laser_gimbal, &current_angle);
        LOG_I("Current angle: yaw=%.2f°, pitch=%.2f°", current_angle.yaw, current_angle.pitch);
    }

    /* 2. 测试坐标控制 */
    LOG_I("Test 2: Coordinate control");
    
    // 测试点1: 前方5米，右侧2米
    result = laser_gimbal_point_to_coordinate(&g_laser_gimbal, 5.4f, 5.0f, 2.0f);
    if (result == RT_EOK)
    {
        LOG_I("Pointed to: distance=5.4m, x=5.0m, y=2.0m");
        rt_thread_mdelay(2000);
    }

    // 测试点2: 前方10米，左侧3米
    result = laser_gimbal_point_to_coordinate(&g_laser_gimbal, 10.4f, 10.0f, -3.0f);
    if (result == RT_EOK)
    {
        LOG_I("Pointed to: distance=10.4m, x=10.0m, y=-3.0m");
        rt_thread_mdelay(2000);
    }

    /* 3. 测试三维坐标控制 */
    LOG_I("Test 3: 3D coordinate control");
    laser_coordinate_t target_3d = {8.0f, 3.0f, 2.0f}; // x=8m, y=3m, z=2m高度
    result = laser_gimbal_point_to_3d_coordinate(&g_laser_gimbal, target_3d);
    if (result == RT_EOK)
    {
        LOG_I("Pointed to 3D coordinate: (%.1f, %.1f, %.1f)", target_3d.x, target_3d.y, target_3d.z);
        rt_thread_mdelay(2000);
    }

    /* 4. 回零测试 */
    LOG_I("Test 4: Homing");
    result = laser_gimbal_home(&g_laser_gimbal);
    if (result == RT_EOK)
    {
        LOG_I("Homing completed");
        rt_thread_mdelay(1000);
    }

    LOG_I("=== Basic Function Test Completed ===");
}

/**
 * @brief 扫射模式测试
 */
static void laser_gimbal_scan_test(void)
{
    LOG_I("=== Starting Scan Pattern Test ===");

    /* 水平扫射 */
    LOG_I("Horizontal scan test");
    for (int i = -30; i <= 30; i += 10)
    {
        laser_gimbal_set_angle(&g_laser_gimbal, (float)i, 0.0f);
        rt_thread_mdelay(500);
        LOG_I("Yaw angle: %d°", i);
    }

    /* 垂直扫射 */
    LOG_I("Vertical scan test");
    laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, 0.0f); // 回到中心
    rt_thread_mdelay(1000);
    
    for (int i = -15; i <= 45; i += 15)
    {
        laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, (float)i);
        rt_thread_mdelay(500);
        LOG_I("Pitch angle: %d°", i);
    }

    /* 回零 */
    laser_gimbal_home(&g_laser_gimbal);
    
    LOG_I("=== Scan Pattern Test Completed ===");
}

/**
 * @brief 角度控制演示测试
 */
static void laser_gimbal_angle_control_demo(void)
{
    LOG_I("=== Starting Angle Control Demonstration ===");

    /* 1. 基本角度控制 */
    LOG_I("Demo 1: Basic angle control");
    
    laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, 0.0f);
    LOG_I("Set to home position (0°, 0°)");
    rt_thread_mdelay(2000);
    
    laser_gimbal_set_angle(&g_laser_gimbal, 30.0f, 15.0f);
    LOG_I("Set to (30°, 15°)");
    rt_thread_mdelay(2000);
    
    laser_gimbal_set_angle(&g_laser_gimbal, -45.0f, -10.0f);
    LOG_I("Set to (-45°, -10°)");
    rt_thread_mdelay(2000);

    /* 2. 单轴控制演示 */
    LOG_I("Demo 2: Single axis control");
    
    laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, 0.0f);
    rt_thread_mdelay(1000);
    
    LOG_I("Yaw only control (pitch stays at 0°):");
    for (int yaw = -30; yaw <= 30; yaw += 15)
    {
        laser_gimbal_set_yaw(&g_laser_gimbal, (float)yaw);
        LOG_I("  Yaw = %d°", yaw);
        rt_thread_mdelay(800);
    }
    
    LOG_I("Pitch only control (yaw stays at 30°):");
    for (int pitch = -15; pitch <= 30; pitch += 15)
    {
        laser_gimbal_set_pitch(&g_laser_gimbal, (float)pitch);
        LOG_I("  Pitch = %d°", pitch);
        rt_thread_mdelay(800);
    }

    /* 3. 相对角度调整演示 */
    LOG_I("Demo 3: Relative angle adjustment");
    
    laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, 0.0f);
    rt_thread_mdelay(1000);
    
    LOG_I("Starting from (0°, 0°), making relative adjustments:");
    
    laser_gimbal_adjust_angle(&g_laser_gimbal, 15.0f, 0.0f);
    LOG_I("  Adjust +15° yaw");
    rt_thread_mdelay(1000);
    
    laser_gimbal_adjust_angle(&g_laser_gimbal, 0.0f, 10.0f);
    LOG_I("  Adjust +10° pitch");
    rt_thread_mdelay(1000);
    
    laser_gimbal_adjust_angle(&g_laser_gimbal, -10.0f, -5.0f);
    LOG_I("  Adjust -10° yaw, -5° pitch");
    rt_thread_mdelay(1000);

    /* 4. 速度控制演示 */
    LOG_I("Demo 4: Speed controlled movement");
    
    laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, 0.0f);
    rt_thread_mdelay(1000);
    
    LOG_I("Normal speed move to (45°, 20°):");
    laser_gimbal_set_angle(&g_laser_gimbal, 45.0f, 20.0f);
    rt_thread_mdelay(3000);
    
    LOG_I("Slow speed move to (-45°, -20°):");
    laser_gimbal_smooth_move_to_angle(&g_laser_gimbal, -45.0f, -20.0f, 0.3f);
    rt_thread_mdelay(5000);
    
    LOG_I("Fast speed move to (0°, 0°):");
    laser_gimbal_smooth_move_to_angle(&g_laser_gimbal, 0.0f, 0.0f, 1.8f);
    rt_thread_mdelay(2000);

    /* 5. 角度查询演示 */
    LOG_I("Demo 5: Angle reading");
    
    laser_gimbal_angle_t current_angle;
    float yaw, pitch;
    
    if (laser_gimbal_get_current_angle(&g_laser_gimbal, &current_angle) == RT_EOK)
    {
        LOG_I("Current angle (structure): yaw=%.2f°, pitch=%.2f°", 
              current_angle.yaw, current_angle.pitch);
    }
    
    if (laser_gimbal_get_yaw(&g_laser_gimbal, &yaw) == RT_EOK &&
        laser_gimbal_get_pitch(&g_laser_gimbal, &pitch) == RT_EOK)
    {
        LOG_I("Current angle (separate): yaw=%.2f°, pitch=%.2f°", yaw, pitch);
    }

    /* 回零 */
    laser_gimbal_home(&g_laser_gimbal);
    LOG_I("=== Angle Control Demonstration Completed ===");
}
static void laser_gimbal_precision_test(void)
{
    LOG_I("=== Starting Precision Test ===");

    /* 测试点数组 (距离, x, y) */
    struct {
        float distance;
        float x;
        float y;
        const char* description;
    } test_points[] = {
        {3.0f, 3.0f, 0.0f, "Front 3m"},
        {5.0f, 4.0f, 3.0f, "Front-right 5m"},
        {7.0f, 0.0f, 7.0f, "Right 7m"},
        {4.0f, -2.0f, 3.0f, "Front-left 4m"},
        {6.0f, -4.0f, -4.0f, "Back-left 6m"}
    };

    size_t test_count = sizeof(test_points) / sizeof(test_points[0]);

    for (size_t i = 0; i < test_count; i++)
    {
        LOG_I("Test %d: %s", i+1, test_points[i].description);
        
        rt_err_t result = laser_gimbal_point_to_coordinate(&g_laser_gimbal,
                                                         test_points[i].distance,
                                                         test_points[i].x,
                                                         test_points[i].y);
        if (result == RT_EOK)
        {
            laser_gimbal_angle_t angle;
            laser_gimbal_get_current_angle(&g_laser_gimbal, &angle);
            LOG_I("  Target: dist=%.1fm, x=%.1fm, y=%.1fm -> Angle: yaw=%.2f°, pitch=%.2f°",
                  test_points[i].distance, test_points[i].x, test_points[i].y,
                  angle.yaw, angle.pitch);
        }
        else
        {
            LOG_E("  Failed to point to target: %d", result);
        }

        rt_thread_mdelay(1500); // 等待稳定
    }

    laser_gimbal_home(&g_laser_gimbal);
    LOG_I("=== Precision Test Completed ===");
}

/**
 * @brief 获取状态信息
 */
static void laser_gimbal_show_status(void)
{
    laser_gimbal_angle_t current_angle;
    laser_gimbal_state_t state;
    uint32_t cmd_count, err_count;

    LOG_I("=== Laser Gimbal Status ===");

    /* 获取当前角度 */
    if (laser_gimbal_get_current_angle(&g_laser_gimbal, &current_angle) == RT_EOK)
    {
        LOG_I("Current angle: yaw=%.2f°, pitch=%.2f°", current_angle.yaw, current_angle.pitch);
    }

    /* 获取状态 */
    state = laser_gimbal_get_state(&g_laser_gimbal);
    const char* state_str[] = {"IDLE", "MOVING", "TRACKING", "ERROR"};
    LOG_I("State: %s", state_str[state]);

    /* 获取统计信息 */
    laser_gimbal_get_statistics(&g_laser_gimbal, &cmd_count, &err_count);
    LOG_I("Statistics: Commands=%d, Errors=%d", cmd_count, err_count);

    /* 获取电机状态 */
    LOG_I("Motor status:");
    ms4010_manager_get_all_status(&g_motor_manager);
}

/* MSH命令实现 */

/**
 * @brief 激光云台初始化命令
 */
static void laser_init(int argc, char **argv)
{
    rt_err_t result = laser_gimbal_system_init();
    if (result == RT_EOK)
    {
        LOG_D("Laser gimbal system initialized successfully\n");
    }
    else
    {
        LOG_D("Failed to initialize laser gimbal system: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_init, Initialize laser gimbal system);

/**
 * @brief 激光云台指向命令
 */
static void laser_point(int argc, char **argv)
{
    if (argc != 4)
    {
        LOG_D("Usage: laser_point <distance> <x> <y>\n");
        LOG_D("Example: laser_point 10.0 8.0 3.0\n");
        return;
    }

    float distance = atof(argv[1]);
    float x = atof(argv[2]);
    float y = atof(argv[3]);

    rt_err_t result = laser_gimbal_point_to_coordinate(&g_laser_gimbal, distance, x, y);
    if (result == RT_EOK)
    {
        LOG_D("Pointing to: distance=%.2fm, x=%.2fm, y=%.2fm\n", distance, x, y);
    }
    else
    {
        LOG_D("Failed to point to coordinate: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_point, Point laser to coordinate: laser_point <distance> <x> <y>);

/**
 * @brief 激光云台角度控制命令
 */
static void laser_angle(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: laser_angle <yaw> <pitch>\n");
        LOG_D("Example: laser_angle 30.0 15.0\n");
        return;
    }

    float yaw = atof(argv[1]);
    float pitch = atof(argv[2]);

    rt_err_t result = laser_gimbal_set_angle(&g_laser_gimbal, yaw, pitch);
    if (result == RT_EOK)
    {
        LOG_D("Set angle: yaw=%.2f°, pitch=%.2f°\n", yaw, pitch);
    }
    else
    {
        LOG_D("Failed to set angle: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_angle, Set laser gimbal angle: laser_angle <yaw> <pitch>);

/**
 * @brief 激光云台单独yaw控制命令
 */
static void laser_yaw(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: laser_yaw <yaw>\n");
        LOG_D("Example: laser_yaw 30.0\n");
        return;
    }

    float yaw = atof(argv[1]);

    rt_err_t result = laser_gimbal_set_yaw(&g_laser_gimbal, yaw);
    if (result == RT_EOK)
    {
        LOG_D("Set yaw angle: %.2f° (pitch unchanged)\n", yaw);
    }
    else
    {
        LOG_D("Failed to set yaw angle: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_yaw, Set laser gimbal yaw angle only: laser_yaw <yaw>);

/**
 * @brief 激光云台单独pitch控制命令
 */
static void laser_pitch(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: laser_pitch <pitch>\n");
        LOG_D("Example: laser_pitch 15.0\n");
        return;
    }

    float pitch = atof(argv[1]);

    rt_err_t result = laser_gimbal_set_pitch(&g_laser_gimbal, pitch);
    if (result == RT_EOK)
    {
        LOG_D("Set pitch angle: %.2f° (yaw unchanged)\n", pitch);
    }
    else
    {
        LOG_D("Failed to set pitch angle: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_pitch, Set laser gimbal pitch angle only: laser_pitch <pitch>);

/**
 * @brief 激光云台相对角度调整命令
 */
static void laser_adjust(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: laser_adjust <yaw_delta> <pitch_delta>\n");
        LOG_D("Example: laser_adjust 10.0 -5.0  (yaw +10°, pitch -5°)\n");
        return;
    }

    float yaw_delta = atof(argv[1]);
    float pitch_delta = atof(argv[2]);

    rt_err_t result = laser_gimbal_adjust_angle(&g_laser_gimbal, yaw_delta, pitch_delta);
    if (result == RT_EOK)
    {
        LOG_D("Adjusted angles: yaw %+.2f°, pitch %+.2f°\n", yaw_delta, pitch_delta);
    }
    else
    {
        LOG_D("Failed to adjust angles: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_adjust, Adjust laser gimbal angles relatively: laser_adjust <yaw_delta> <pitch_delta>);

/**
 * @brief 激光云台平滑移动命令
 */
static void laser_smooth(int argc, char **argv)
{
    if (argc < 3 || argc > 4)
    {
        LOG_D("Usage: laser_smooth <yaw> <pitch> [speed_factor]\n");
        LOG_D("Example: laser_smooth 30.0 15.0 0.5  (half speed)\n");
        LOG_D("  speed_factor: 0.1-2.0 (default: 1.0)\n");
        return;
    }

    float yaw = atof(argv[1]);
    float pitch = atof(argv[2]);
    float speed_factor = (argc == 4) ? atof(argv[3]) : 1.0f;

    rt_err_t result = laser_gimbal_smooth_move_to_angle(&g_laser_gimbal, yaw, pitch, speed_factor);
    if (result == RT_EOK)
    {
        LOG_D("Smooth move to: yaw=%.2f°, pitch=%.2f°, speed=%.1fx\n", 
              yaw, pitch, speed_factor);
    }
    else
    {
        LOG_D("Failed to smooth move: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_smooth, Smooth move to angle with speed control: laser_smooth <yaw> <pitch> [speed_factor]);

/**
 * @brief 获取当前角度命令
 */
static void laser_get_angle(int argc, char **argv)
{
    laser_gimbal_angle_t current_angle;
    rt_err_t result = laser_gimbal_get_current_angle(&g_laser_gimbal, &current_angle);
    
    if (result == RT_EOK)
    {
        LOG_D("Current angles: yaw=%.2f°, pitch=%.2f°\n", 
              current_angle.yaw, current_angle.pitch);
        
        /* 也可以单独获取 */
        float yaw, pitch;
        laser_gimbal_get_yaw(&g_laser_gimbal, &yaw);
        laser_gimbal_get_pitch(&g_laser_gimbal, &pitch);
        LOG_D("Separate get: yaw=%.2f°, pitch=%.2f°\n", yaw, pitch);
    }
    else
    {
        LOG_D("Failed to get current angles: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_get_angle, Get current laser gimbal angles);

/**
 * @brief 激光云台回零命令
 */
static void laser_home(int argc, char **argv)
{
    rt_err_t result = laser_gimbal_home(&g_laser_gimbal);
    if (result == RT_EOK)
    {
        LOG_D("Gimbal homing completed\n");
    }
    else
    {
        LOG_D("Failed to home gimbal: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_home, Home laser gimbal);

/**
 * @brief 激光云台状态查看命令
 */
static void laser_status(int argc, char **argv)
{
    laser_gimbal_show_status();
}
MSH_CMD_EXPORT(laser_status, Show laser gimbal status);

/**
 * @brief 激光云台测试命令
 */
static void laser_test(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("Usage: laser_test <basic|scan|precision|angle>\n");
        return;
    }

    if (rt_strcmp(argv[1], "basic") == 0)
    {
        laser_gimbal_basic_test();
    }
    else if (rt_strcmp(argv[1], "scan") == 0)
    {
        laser_gimbal_scan_test();
    }
    else if (rt_strcmp(argv[1], "precision") == 0)
    {
        laser_gimbal_precision_test();
    }
    else if (rt_strcmp(argv[1], "angle") == 0)
    {
        laser_gimbal_angle_control_demo();
    }
    else
    {
        LOG_D("Unknown test type: %s\n", argv[1]);
        LOG_D("Available tests: basic, scan, precision, angle\n");
    }
}
MSH_CMD_EXPORT(laser_test, Run laser gimbal tests: laser_test <basic|scan|precision|angle>);

/**
 * @brief 激光云台停止命令
 */
static void laser_stop(int argc, char **argv)
{
    rt_err_t result = laser_gimbal_stop(&g_laser_gimbal);
    if (result == RT_EOK)
    {
        LOG_D("Gimbal stopped\n");
    }
    else
    {
        LOG_D("Failed to stop gimbal: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_stop, Stop laser gimbal motion);

/* ======================== 速度控制命令 ======================== */

/**
 * @brief 设置yaw轴旋转速度
 */
static void laser_yaw_speed(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: laser_yaw_speed <speed>\n");
        LOG_D("  speed: Rotation speed in degrees/second\n");
        LOG_D("         Positive: Turn right, Negative: Turn left, 0: Stop\n");
        LOG_D("Example: laser_yaw_speed 30.0  (turn right at 30°/s)\n");
        LOG_D("         laser_yaw_speed -15.0 (turn left at 15°/s)\n");
        LOG_D("         laser_yaw_speed 0     (stop rotation)\n");
        return;
    }

    float speed = atof(argv[1]);
    
    rt_err_t result = laser_gimbal_set_yaw_speed(&g_laser_gimbal, speed);
    if (result == RT_EOK)
    {
        LOG_D("Yaw speed set to %.1f°/s\n", speed);
        if (speed > 0)
            LOG_D("Turning RIGHT at %.1f°/s\n", speed);
        else if (speed < 0)
            LOG_D("Turning LEFT at %.1f°/s\n", -speed);
        else
            LOG_D("Yaw rotation STOPPED\n");
    }
    else
    {
        LOG_D("Failed to set yaw speed: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_yaw_speed, Set yaw axis rotation speed: laser_yaw_speed <speed>);

/**
 * @brief 设置pitch轴旋转速度
 */
static void laser_pitch_speed(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: laser_pitch_speed <speed>\n");
        LOG_D("  speed: Rotation speed in degrees/second\n");
        LOG_D("         Positive: Turn up, Negative: Turn down, 0: Stop\n");
        LOG_D("Example: laser_pitch_speed 20.0  (turn up at 20°/s)\n");
        LOG_D("         laser_pitch_speed -10.0 (turn down at 10°/s)\n");
        LOG_D("         laser_pitch_speed 0     (stop rotation)\n");
        return;
    }

    float speed = atof(argv[1]);
    
    rt_err_t result = laser_gimbal_set_pitch_speed(&g_laser_gimbal, speed);
    if (result == RT_EOK)
    {
        LOG_D("Pitch speed set to %.1f°/s\n", speed);
        if (speed > 0)
            LOG_D("Turning UP at %.1f°/s\n", speed);
        else if (speed < 0)
            LOG_D("Turning DOWN at %.1f°/s\n", -speed);
        else
            LOG_D("Pitch rotation STOPPED\n");
    }
    else
    {
        LOG_D("Failed to set pitch speed: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_pitch_speed, Set pitch axis rotation speed: laser_pitch_speed <speed>);

/**
 * @brief 同时设置yaw和pitch轴旋转速度
 */
static void laser_angular_velocity(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: laser_angular_velocity <yaw_speed> <pitch_speed>\n");
        LOG_D("  yaw_speed: Yaw rotation speed (°/s, +right, -left)\n");
        LOG_D("  pitch_speed: Pitch rotation speed (°/s, +up, -down)\n");
        LOG_D("Example: laser_angular_velocity 30.0 -15.0  (right+down)\n");
        LOG_D("         laser_angular_velocity 0 0         (stop all)\n");
        return;
    }

    float yaw_speed = atof(argv[1]);
    float pitch_speed = atof(argv[2]);
    
    rt_err_t result = laser_gimbal_set_angular_velocity(&g_laser_gimbal, yaw_speed, pitch_speed);
    if (result == RT_EOK)
    {
        LOG_D("Angular velocity set: Yaw=%.1f°/s, Pitch=%.1f°/s\n", yaw_speed, pitch_speed);
        if (yaw_speed == 0.0f && pitch_speed == 0.0f)
        {
            LOG_D("All rotation STOPPED\n");
        }
        else
        {
            LOG_D("Direction: ");
            if (yaw_speed > 0) LOG_D("RIGHT ");
            else if (yaw_speed < 0) LOG_D("LEFT ");
            if (pitch_speed > 0) LOG_D("UP");
            else if (pitch_speed < 0) LOG_D("DOWN");
            LOG_D("\n");
        }
    }
    else
    {
        LOG_D("Failed to set angular velocity: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_angular_velocity, Set both axis rotation speeds: laser_angular_velocity <yaw_speed> <pitch_speed>);

/**
 * @brief 获取当前旋转速度
 */
static void laser_get_speed(int argc, char **argv)
{
    float yaw_speed, pitch_speed;
    
    rt_err_t result1 = laser_gimbal_get_yaw_speed(&g_laser_gimbal, &yaw_speed);
    rt_err_t result2 = laser_gimbal_get_pitch_speed(&g_laser_gimbal, &pitch_speed);
    
    if (result1 == RT_EOK && result2 == RT_EOK)
    {
        LOG_D("Current rotation speeds:\n");
        LOG_D("  Yaw:   %.1f°/s", yaw_speed);
        if (yaw_speed > 0) LOG_D(" (turning RIGHT)");
        else if (yaw_speed < 0) LOG_D(" (turning LEFT)");
        else LOG_D(" (STOPPED)");
        LOG_D("\n");
        
        LOG_D("  Pitch: %.1f°/s", pitch_speed);
        if (pitch_speed > 0) LOG_D(" (turning UP)");
        else if (pitch_speed < 0) LOG_D(" (turning DOWN)");
        else LOG_D(" (STOPPED)");
        LOG_D("\n");
        
        if (yaw_speed == 0.0f && pitch_speed == 0.0f)
        {
            LOG_D("✅ Gimbal is stationary\n");
        }
        else
        {
            LOG_D("🔄 Gimbal is rotating\n");
        }
    }
    else
    {
        LOG_D("Failed to get rotation speeds: yaw=%d, pitch=%d\n", result1, result2);
    }
}
MSH_CMD_EXPORT(laser_get_speed, Get current rotation speeds);

/**
 * @brief 停止速度控制，回到位置控制模式
 */
static void laser_stop_velocity(int argc, char **argv)
{
    rt_err_t result = laser_gimbal_stop_velocity_control(&g_laser_gimbal);
    if (result == RT_EOK)
    {
        LOG_D("✅ Velocity control stopped - returned to position control mode\n");
    }
    else
    {
        LOG_D("❌ Failed to stop velocity control: %d\n", result);
    }
}
MSH_CMD_EXPORT(laser_stop_velocity, Stop velocity control and return to position mode);

/* ======================== 校准相关命令 ======================== */

/**
 * @brief 激光云台校准模式控制命令
 */
static void cmd_laser_calibration(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: laser_calibration <on|off>\n");
        LOG_D("  on  - Enter calibration mode (direct motor control)\n");
        LOG_D("  off - Exit calibration mode (normal operation)\n");
        return;
    }

    bool enable = (rt_strcmp(argv[1], "on") == 0);
    
    rt_err_t result = laser_gimbal_calibration_mode(&g_laser_gimbal, enable);
    if (result == RT_EOK)
    {
        LOG_D("Calibration mode: %s\n", enable ? "ENABLED" : "DISABLED");
        if (enable)
        {
            LOG_D("📝 Calibration Commands:\n");
            LOG_D("  laser_cal_set <yaw> <pitch>     - Set motor angles directly\n");
            LOG_D("  laser_cal_adjust <yaw> <pitch>  - Adjust angles incrementally\n");
            LOG_D("  laser_cal_offset <yaw> <pitch>  - Calculate and set offset\n");
            LOG_D("  laser_cal_save                  - Save calibration to file\n");
        }
    }
    else
    {
        LOG_D("Failed to %s calibration mode: %d\n", enable ? "enable" : "disable", result);
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_calibration, laser_calibration, Control calibration mode: laser_calibration <on|off>);

/**
 * @brief 校准模式下设置电机角度命令
 */
static void cmd_laser_cal_set(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: laser_cal_set <yaw_motor_angle> <pitch_motor_angle>\n");
        LOG_D("Example: laser_cal_set 0.0 0.0\n");
        return;
    }

    float yaw_angle = atof(argv[1]);
    float pitch_angle = atof(argv[2]);

    rt_err_t result = laser_gimbal_calibration_set_motor_angle(&g_laser_gimbal, yaw_angle, pitch_angle);
    if (result == RT_EOK)
    {
        LOG_D("Motor angles set: Yaw=%.2f°, Pitch=%.2f°\n", yaw_angle, pitch_angle);
    }
    else
    {
        LOG_D("Failed to set motor angles: %d\n", result);
        LOG_D("Make sure calibration mode is enabled first!\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_set, laser_cal_set, Set motor angles in calibration mode);

/**
 * @brief 校准模式下微调角度命令
 */
static void cmd_laser_cal_adjust(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: laser_cal_adjust <yaw_delta> <pitch_delta>\n");
        LOG_D("Example: laser_cal_adjust 1.0 -0.5  (yaw +1°, pitch -0.5°)\n");
        return;
    }

    float yaw_delta = atof(argv[1]);
    float pitch_delta = atof(argv[2]);

    rt_err_t result = laser_gimbal_calibration_adjust_angle(&g_laser_gimbal, yaw_delta, pitch_delta);
    if (result == RT_EOK)
    {
        float yaw_raw, pitch_raw;
        laser_gimbal_get_motor_raw_angle(&g_laser_gimbal, &yaw_raw, &pitch_raw);
        LOG_D("Angles adjusted by: Yaw %+.2f°, Pitch %+.2f°\n", yaw_delta, pitch_delta);
        LOG_D("Current motor angles: Yaw=%.2f°, Pitch=%.2f°\n", yaw_raw, pitch_raw);
    }
    else
    {
        LOG_D("Failed to adjust angles: %d\n", result);
        LOG_D("Make sure calibration mode is enabled first!\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_adjust, laser_cal_adjust, Adjust motor angles incrementally);

/**
 * @brief 计算角度偏移命令
 */
static void cmd_laser_cal_offset(int argc, char **argv)
{
    float target_yaw = 0.0f, target_pitch = 0.0f;
    
    if (argc == 3)
    {
        target_yaw = atof(argv[1]);
        target_pitch = atof(argv[2]);
    }
    else if (argc != 1)
    {
        LOG_D("Usage: laser_cal_offset [target_yaw] [target_pitch]\n");
        LOG_D("Example: laser_cal_offset 0.0 0.0  (default if no params)\n");
        LOG_D("Call this when gimbal is pointing to the desired zero position\n");
        return;
    }

    rt_err_t result = laser_gimbal_calibrate_offset(&g_laser_gimbal, target_yaw, target_pitch);
    if (result == RT_EOK)
    {
        LOG_D("✅ Offset calibration completed!\n");
        LOG_D("Use 'laser_cal_save' to save these settings\n");
    }
    else
    {
        LOG_D("Failed to calibrate offset: %d\n", result);
        LOG_D("Make sure calibration mode is enabled and gimbal is positioned correctly!\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_offset, laser_cal_offset, Calculate angle offset from current position);

/**
 * @brief 保存校准参数命令
 */
static void cmd_laser_cal_save(int argc, char **argv)
{
    const char *filename = (argc > 1) ? argv[1] : RT_NULL;
    
    rt_err_t result = laser_gimbal_save_calibration(&g_laser_gimbal, filename);
    if (result == RT_EOK)
    {
        LOG_D("✅ Calibration parameters saved successfully!\n");
    }
    else
    {
        LOG_D("Failed to save calibration: %d\n", result);
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_save, laser_cal_save, Save calibration parameters to file);

/**
 * @brief 加载校准参数命令
 */
static void cmd_laser_cal_load(int argc, char **argv)
{
    const char *filename = (argc > 1) ? argv[1] : RT_NULL;
    
    rt_err_t result = laser_gimbal_load_calibration(&g_laser_gimbal, filename);
    if (result == RT_EOK)
    {
        LOG_D("✅ Calibration parameters loaded successfully!\n");
    }
    else
    {
        LOG_D("Failed to load calibration: %d\n", result);
        LOG_D("Use 'laser_cal_save' to create calibration file first\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_load, laser_cal_load, Load calibration parameters from file);

/**
 * @brief 显示当前电机原始角度命令
 */
static void cmd_laser_cal_show(int argc, char **argv)
{
    float yaw_raw, pitch_raw;
    rt_err_t result = laser_gimbal_get_motor_raw_angle(&g_laser_gimbal, &yaw_raw, &pitch_raw);
    
    if (result == RT_EOK)
    {
        LOG_D("=== Current Motor Raw Angles ===\n");
        LOG_D("Yaw motor:   %.2f°\n", yaw_raw);
        LOG_D("Pitch motor: %.2f°\n", pitch_raw);
        
        LOG_D("\n=== Current Offsets ===\n");
        LOG_D("Yaw offset:   %.2f°\n", g_laser_gimbal.yaw_offset);
        LOG_D("Pitch offset: %.2f°\n", g_laser_gimbal.pitch_offset);
        LOG_D("Yaw reverse:  %s\n", g_laser_gimbal.yaw_reverse ? "Yes" : "No");
        LOG_D("Pitch reverse: %s\n", g_laser_gimbal.pitch_reverse ? "Yes" : "No");
        
        LOG_D("\n=== Calibration Mode ===\n");
        LOG_D("Status: %s\n", g_laser_gimbal.calibration_mode ? "ENABLED" : "DISABLED");
    }
    else
    {
        LOG_D("Failed to get motor angles: %d\n", result);
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_show, laser_cal_show, Show current motor angles and calibration info);

/**
 * @brief 激光云台校准向导
 */
static void cmd_laser_calibration_wizard(int argc, char **argv)
{
    LOG_D("🎯 === Laser Gimbal Calibration Wizard (Auto-Save Mode) ===\n\n");
    
    LOG_D("📋 Step 1: Prepare for calibration\n");
    LOG_D("  1. Make sure gimbal is powered and initialized\n");
    LOG_D("  2. Prepare a reference target (laser pointer spot on wall)\n");
    LOG_D("  3. Measure distance to target (recommend 3-5 meters)\n\n");
    
    LOG_D("⚙️  Step 2: Configure monitoring (optional)\n");
    LOG_D("  laser_cal_config 0.5 3.0 1    # 0.5° threshold, 3s stable, auto-save on\n");
    LOG_D("  Current settings: %.1f° threshold, %.1fs stable time, auto-save %s\n\n",
              g_laser_gimbal.angle_stable_threshold,
              g_laser_gimbal.angle_stable_time_ms / 1000.0f,
              g_laser_gimbal.auto_save_enabled ? "ON" : "OFF");
    
    LOG_D("🔓 Step 3: Enter calibration mode\n");
    LOG_D("  laser_calibration on\n");
    LOG_D("  → Motors will turn OFF for manual adjustment\n");
    LOG_D("  → Real-time angle monitoring will start\n\n");
    
    LOG_D("🖐️ Step 4: Manual adjustment\n");
    LOG_D("  1. Use your hands to adjust gimbal position\n");
    LOG_D("  2. Point laser exactly to target center (0°, 0° position)\n");
    LOG_D("  3. Watch the console for real-time angle updates\n");
    LOG_D("  4. Hold position steady until auto-save triggers\n\n");
    
    LOG_D("⏱️  Step 5: Wait for auto-save\n");
    LOG_D("  → System monitors angle stability automatically\n");
    LOG_D("  → When angles are stable for %.1fs, calibration saves automatically\n",
              g_laser_gimbal.angle_stable_time_ms / 1000.0f);
    LOG_D("  → You'll see: \"✅ Auto-calibration completed!\"\n\n");
    
    LOG_D("✅ Step 6: Test calibration\n");
    LOG_D("  1. Exit calibration mode: laser_calibration off\n");
    LOG_D("  2. Test: laser_angle 0 0 (should point to target center)\n");
    LOG_D("  3. Test: laser_angle 30 0 (should point 30° right of target)\n\n");
    
    LOG_D("� Manual override commands (if needed):\n");
    LOG_D("  laser_cal_show                   - Check current angles\n");
    LOG_D("  laser_cal_offset 0 0             - Manual offset calculation\n");
    LOG_D("  laser_cal_save                   - Manual save\n\n");
    
    LOG_D("💡 New workflow is much simpler:\n");
    LOG_D("  1. laser_calibration on         # Motors OFF, monitoring ON\n");
    LOG_D("  2. Adjust by hand to point at target center\n");
    LOG_D("  3. Hold steady for 3 seconds    # Auto-save triggers\n");
    LOG_D("  4. laser_calibration off        # Done!\n");
    LOG_D("  5. Test with laser_angle commands\n\n");
    LOG_D("  laser_cal_save              # Save settings\n");
    LOG_D("  laser_calibration off       # Return to normal mode\n");
    LOG_D("  laser_angle 0 0             # Test - should point forward\n\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_calibration_wizard, laser_cal_wizard, Show calibration wizard guide);

/**
 * @brief 配置校准监控参数
 */
static void cmd_laser_cal_config(int argc, char **argv)
{
    if (argc == 1)
    {
        LOG_D("Current calibration monitor settings:\n");
        LOG_D("  Stable threshold: %.2f°\n", g_laser_gimbal.angle_stable_threshold);
        LOG_D("  Stable time: %.1fs\n", g_laser_gimbal.angle_stable_time_ms / 1000.0f);
        LOG_D("  Auto-save: %s\n", g_laser_gimbal.auto_save_enabled ? "Enabled" : "Disabled");
        LOG_D("\nUsage: laser_cal_config <threshold> <time_sec> <auto_save>\n");
        LOG_D("  threshold: Angle stable threshold in degrees (0.1-2.0)\n");
        LOG_D("  time_sec: Stable time in seconds (1.0-10.0)\n");
        LOG_D("  auto_save: Auto-save when stable (0=off, 1=on)\n");
        LOG_D("Example: laser_cal_config 0.5 3.0 1\n");
        return;
    }

    if (argc != 4)
    {
        LOG_D("Error: Invalid argument count\n");
        LOG_D("Usage: laser_cal_config <threshold> <time_sec> <auto_save>\n");
        return;
    }

    float threshold = atof(argv[1]);
    float time_sec = atof(argv[2]);
    int auto_save = atoi(argv[3]);

    if (threshold < 0.1f || threshold > 2.0f)
    {
        LOG_D("Error: Threshold must be between 0.1° and 2.0°\n");
        return;
    }

    if (time_sec < 1.0f || time_sec > 10.0f)
    {
        LOG_D("Error: Time must be between 1.0s and 10.0s\n");
        return;
    }

    uint32_t time_ms = (uint32_t)(time_sec * 1000);
    bool enable_auto_save = (auto_save != 0);

    rt_err_t result = laser_gimbal_set_calibration_monitor_params(&g_laser_gimbal, 
                                                                  threshold, 
                                                                  time_ms, 
                                                                  enable_auto_save);
    if (result == RT_EOK)
    {
        LOG_D("✅ Calibration monitor parameters updated successfully\n");
    }
    else
    {
        LOG_D("❌ Failed to update calibration monitor parameters\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_config, laser_cal_config, Configure calibration monitor parameters);

/**
 * @brief 验证校准偏移是否生效
 */
static void cmd_laser_cal_verify(int argc, char **argv)
{
    LOG_D("=== Calibration Verification Test ===\n\n");
    
    /* 显示当前偏移设置 */
    LOG_D("📋 Current calibration settings:\n");
    LOG_D("  Yaw offset: %.3f°\n", g_laser_gimbal.yaw_offset);
    LOG_D("  Pitch offset: %.3f°\n", g_laser_gimbal.pitch_offset);
    LOG_D("  Yaw reverse: %s\n", g_laser_gimbal.yaw_reverse ? "Yes" : "No");
    LOG_D("  Pitch reverse: %s\n", g_laser_gimbal.pitch_reverse ? "Yes" : "No");
    
    /* 测试角度转换 */
    LOG_D("\n🧪 Testing angle conversion:\n");
    
    float test_angles[][2] = {
        {0.0f, 0.0f},     // 零位测试
        {30.0f, 15.0f},   // 正角度测试
        {-30.0f, -15.0f}, // 负角度测试
    };
    
    for (int i = 0; i < 3; i++)
    {
        float logic_yaw = test_angles[i][0];
        float logic_pitch = test_angles[i][1];
        
        /* 计算应该发送给电机的角度 */
        float motor_yaw = logic_yaw + g_laser_gimbal.yaw_offset;
        float motor_pitch = logic_pitch + g_laser_gimbal.pitch_offset;
        
        if (g_laser_gimbal.yaw_reverse)
            motor_yaw = -motor_yaw;
        if (g_laser_gimbal.pitch_reverse)
            motor_pitch = -motor_pitch;
        
        LOG_D("  Logic angle: (%.1f°, %.1f°) → Motor angle: (%.3f°, %.3f°)\n", 
                   logic_yaw, logic_pitch, motor_yaw, motor_pitch);
    }
    
    /* 实际测试一个角度 */
    LOG_D("\n🎯 Real test: Setting logic angle (0°, 0°)...\n");
    rt_err_t result = laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, 0.0f);
    if (result == RT_EOK)
    {
        LOG_D("✅ Command sent successfully\n");
        LOG_D("💡 If calibration is correct, gimbal should point to calibrated zero position\n");
    }
    else
    {
        LOG_D("❌ Failed to set angle: %d\n", result);
    }
    
    LOG_D("\n💾 Tip: Use 'laser_cal_show' to check current motor angles\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_verify, laser_cal_verify, Verify calibration offset effectiveness);

/**
 * @brief 查看校准参数存储状态命令
 */
static void cmd_laser_cal_storage(int argc, char **argv)
{
    laser_gimbal_show_calibration_storage();
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_storage, laser_cal_storage, Show calibration parameter storage status);

/**
 * @brief 重置校准参数存储命令
 */
static void cmd_laser_cal_reset(int argc, char **argv)
{
    if (argc == 2 && rt_strcmp(argv[1], "confirm") == 0)
    {
        laser_gimbal_reset_calibration_storage();
    }
    else
    {
        LOG_D("⚠️  WARNING: This will permanently delete all calibration parameters!\n");
        LOG_D("Usage: laser_cal_reset confirm\n");
        LOG_D("Add 'confirm' parameter to proceed with reset.\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_cal_reset, laser_cal_reset, Reset calibration parameter storage (requires confirm));

/**
 * @brief 测试home点坐标转换命令
 */
static void cmd_laser_home_test(int argc, char **argv)
{
    LOG_D("=== Home Point Test ===\n");
    
    // 测试坐标(0,0,z)的转换
    laser_coordinate_t home_coord = {0.0f, 0.0f, 1.0f};  // z可以是任意值
    laser_gimbal_angle_t angle;
    
    rt_err_t result = laser_gimbal_coordinate_to_angle(home_coord, g_laser_gimbal.gimbal_height, &angle);
    if (result == RT_EOK)
    {
        LOG_D("✅ Home coordinate (0,0,%.1f) -> Angle (%.2f°, %.2f°)\n", 
                   home_coord.z, angle.yaw, angle.pitch);
        LOG_D("Expected: Yaw=0°, Pitch=0° (should match!)\n");
        
        // 实际指向home点
        result = laser_gimbal_point_to_3d_coordinate(&g_laser_gimbal, home_coord);
        if (result == RT_EOK)
        {
            LOG_D("✅ Successfully pointed to home position\n");
        }
        else
        {
            LOG_D("❌ Failed to point to home position\n");
        }
    }
    else
    {
        LOG_D("❌ Failed to convert home coordinate to angle\n");
    }
    
    LOG_D("=== Test completed ===\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_home_test, laser_home_test, Test home point coordinate conversion);

/* ======================== 激光绘制功能实现 ======================== */

/**
 * @brief 绘制矩形
 * @param center_x 矩形中心X坐标
 * @param center_y 矩形中心Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param distance 绘制距离
 * @param point_count 每边的点数
 * @param delay_ms 每个点停留时间(毫秒)
 */
static void laser_draw_rectangle(float center_x, float center_y, float width, float height, 
                                float distance, int point_count, int delay_ms)
{
    LOG_D("🔳 Drawing rectangle: center(%.1f,%.1f), size(%.1fx%.1f), distance=%.1fm, %d points/edge, %dms delay\n",
               center_x, center_y, width, height, distance, point_count, delay_ms);
    
    float half_width = width / 2.0f;
    float half_height = height / 2.0f;
    
    // 计算矩形四个顶点
    float corners[4][2] = {
        {center_x - half_width, center_y - half_height}, // 左下
        {center_x + half_width, center_y - half_height}, // 右下
        {center_x + half_width, center_y + half_height}, // 右上
        {center_x - half_width, center_y + half_height}  // 左上
    };
    
    // 绘制四条边
    for (int edge = 0; edge < 4; edge++)
    {
        LOG_D("Drawing edge %d...\n", edge + 1);
        
        float start_x = corners[edge][0];
        float start_y = corners[edge][1];
        float end_x = corners[(edge + 1) % 4][0];
        float end_y = corners[(edge + 1) % 4][1];
        
        // 在每条边上绘制点
        for (int i = 0; i <= point_count; i++)
        {
            float t = (float)i / (float)point_count;
            float x = start_x + t * (end_x - start_x);
            float y = start_y + t * (end_y - start_y);
            
            rt_err_t result = laser_gimbal_point_to_coordinate(&g_laser_gimbal, distance, x, y);
            if (result != RT_EOK)
            {
                LOG_D("❌ Failed to point to (%.2f, %.2f)\n", x, y);
            }
            rt_thread_mdelay(delay_ms); // 用户指定的停留时间
        }
    }
    
    LOG_D("✅ Rectangle drawing completed\n");
}

/**
 * @brief 绘制圆形
 * @param center_x 圆心X坐标
 * @param center_y 圆心Y坐标
 * @param radius 半径
 * @param distance 绘制距离
 * @param point_count 圆周上的点数
 * @param delay_ms 每个点停留时间(毫秒)
 */
static void laser_draw_circle(float center_x, float center_y, float radius, 
                             float distance, int point_count, int delay_ms)
{
    LOG_D("⭕ Drawing circle: center(%.1f,%.1f), radius=%.1f, distance=%.1fm, %d points, %dms delay\n",
               center_x, center_y, radius, distance, point_count, delay_ms);
    
    // 绘制圆周上的点
    for (int i = 0; i <= point_count; i++)
    {
        float angle = 2.0f * M_PI * (float)i / (float)point_count;
        float x = center_x + radius * cosf(angle);
        float y = center_y + radius * sinf(angle);
        
        rt_err_t result = laser_gimbal_point_to_coordinate(&g_laser_gimbal, distance, x, y);
        if (result != RT_EOK)
        {
            LOG_D("❌ Failed to point to (%.2f, %.2f)\n", x, y);
        }
        
        rt_thread_mdelay(delay_ms); // 用户指定的停留时间
        
        // 每10个点显示进度
        if (i % 10 == 0 || i == point_count)
        {
            float progress = (float)i / (float)point_count * 100.0f;
            LOG_D("Progress: %.1f%% (%.1f°)\n", progress, angle * 180.0f / M_PI);
        }
    }
    
    LOG_D("✅ Circle drawing completed\n");
}

/**
 * @brief 绘制正弦函数
 * @param amplitude 振幅
 * @param frequency 频率 (周期数)
 * @param phase 相位偏移 (弧度)
 * @param x_start X轴起始位置
 * @param x_end X轴结束位置
 * @param y_offset Y轴偏移
 * @param distance 绘制距离
 * @param point_count 采样点数
 * @param delay_ms 每个点停留时间(毫秒)
 */
static void laser_draw_sine_wave(float amplitude, float frequency, float phase,
                                 float x_start, float x_end, float y_offset,
                                 float distance, int point_count, int delay_ms)
{
    LOG_D("〰️ Drawing sine wave: A=%.1f, f=%.1f, φ=%.1f°, x=[%.1f,%.1f], y_offset=%.1f, %d points, %dms delay\n",
               amplitude, frequency, phase * 180.0f / M_PI, x_start, x_end, y_offset, point_count, delay_ms);
    
    float x_range = x_end - x_start;
    
    // 绘制正弦函数上的点
    for (int i = 0; i <= point_count; i++)
    {
        float t = (float)i / (float)point_count;
        float x = x_start + t * x_range;
        
        // 计算正弦函数值: y = A * sin(2π * f * t + φ) + offset
        float normalized_x = (x - x_start) / x_range; // 将x归一化到[0,1]
        float y = amplitude * sinf(2.0f * M_PI * frequency * normalized_x + phase) + y_offset;
        
        rt_err_t result = laser_gimbal_point_to_coordinate(&g_laser_gimbal, distance, x, y);
        if (result != RT_EOK)
        {
            LOG_D("❌ Failed to point to (%.2f, %.2f)\n", x, y);
        }
        
        rt_thread_mdelay(delay_ms); // 用户指定的停留时间
        
        // 每20个点显示进度
        if (i % 20 == 0 || i == point_count)
        {
            float progress = t * 100.0f;
            LOG_D("Progress: %.1f%% - Point(%.2f, %.2f)\n", progress, x, y);
        }
    }
    
    LOG_D("✅ Sine wave drawing completed\n");
}

/* ======================== MSH绘制命令实现 ======================== */

/**
 * @brief 激光绘制矩形命令
 */
static void cmd_laser_draw_rect(int argc, char **argv)
{
    if (argc < 6 || argc > 8)
    {
        LOG_D("Usage: laser_draw_rect <center_x> <center_y> <width> <height> <distance> [points_per_edge] [delay_ms]\n");
        LOG_D("Example: laser_draw_rect 2.0 1.0 3.0 2.0 5.0 20 50\n");
        LOG_D("  center_x, center_y: Rectangle center coordinates (m)\n");
        LOG_D("  width, height: Rectangle dimensions (m)\n");
        LOG_D("  distance: Drawing distance from gimbal (m)\n");
        LOG_D("  points_per_edge: Points per edge (default: 20)\n");
        LOG_D("  delay_ms: Delay per point in milliseconds (default: 50)\n");
        return;
    }

    float center_x = atof(argv[1]);
    float center_y = atof(argv[2]);
    float width = atof(argv[3]);
    float height = atof(argv[4]);
    float distance = atof(argv[5]);
    
    // 默认参数
    int point_count = (argc > 6) ? atoi(argv[6]) : 20;      // 默认每边20个点
    int delay_ms = (argc > 7) ? atoi(argv[7]) : 50;         // 默认50ms延时

    // 参数验证
    if (width <= 0 || height <= 0 || distance <= 0)
    {
        LOG_D("❌ Error: Width, height, and distance must be positive\n");
        return;
    }
    
    if (point_count < 2 || point_count > 100)
    {
        LOG_D("❌ Error: Points per edge must be between 2 and 100\n");
        return;
    }
    
    if (delay_ms < 10 || delay_ms > 2000)
    {
        LOG_D("❌ Error: Delay must be between 10ms and 2000ms\n");
        return;
    }

    laser_draw_rectangle(center_x, center_y, width, height, distance, point_count, delay_ms);
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_draw_rect, laser_draw_rect, Draw rectangle with laser points);

/**
 * @brief 激光绘制圆形命令
 */
static void cmd_laser_draw_circle(int argc, char **argv)
{
    if (argc < 5 || argc > 7)
    {
        LOG_D("Usage: laser_draw_circle <center_x> <center_y> <radius> <distance> [point_count] [delay_ms]\n");
        LOG_D("Example: laser_draw_circle 3.0 2.0 1.5 6.0 60 30\n");
        LOG_D("  center_x, center_y: Circle center coordinates (m)\n");
        LOG_D("  radius: Circle radius (m)\n");
        LOG_D("  distance: Drawing distance from gimbal (m)\n");
        LOG_D("  point_count: Points on circumference (default: 60)\n");
        LOG_D("  delay_ms: Delay per point in milliseconds (default: 30)\n");
        return;
    }

    float center_x = atof(argv[1]);
    float center_y = atof(argv[2]);
    float radius = atof(argv[3]);
    float distance = atof(argv[4]);
    
    // 默认参数
    int point_count = (argc > 5) ? atoi(argv[5]) : 60;      // 默认圆周60个点
    int delay_ms = (argc > 6) ? atoi(argv[6]) : 30;         // 默认30ms延时

    // 参数验证
    if (radius <= 0 || distance <= 0)
    {
        LOG_D("❌ Error: Radius and distance must be positive\n");
        return;
    }
    
    if (point_count < 2 || point_count > 10000)
    {
        LOG_D("❌ Error: Point count must be between 3 and 10000\n");
        return;
    }
    
    if (delay_ms < 1 || delay_ms > 2000)
    {
        LOG_D("❌ Error: Delay must be between 1ms and 2000ms\n");
        return;
    }

    laser_draw_circle(center_x, center_y, radius, distance, point_count, delay_ms);
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_draw_circle, laser_draw_circle, Draw circle with laser points);

/**
 * @brief 激光绘制正弦波命令
 */
static void cmd_laser_draw_sine(int argc, char **argv)
{
    if (argc < 7 || argc > 10)
    {
        LOG_D("Usage: laser_draw_sine <amplitude> <frequency> <x_start> <x_end> <y_offset> <distance> [phase] [point_count] [delay_ms]\n");
        LOG_D("Example: laser_draw_sine 1.0 2.0 0.0 4.0 2.0 5.0 0.0 100 25\n");
        LOG_D("  amplitude: Wave amplitude (m)\n");
        LOG_D("  frequency: Number of periods in the range\n");
        LOG_D("  x_start, x_end: X range for drawing (m)\n");
        LOG_D("  y_offset: Y axis offset (m)\n");
        LOG_D("  distance: Drawing distance from gimbal (m)\n");
        LOG_D("  phase: Phase offset in degrees (default: 0)\n");
        LOG_D("  point_count: Number of sample points (default: 100)\n");
        LOG_D("  delay_ms: Delay per point in milliseconds (default: 25)\n");
        return;
    }

    float amplitude = atof(argv[1]);
    float frequency = atof(argv[2]);
    float x_start = atof(argv[3]);
    float x_end = atof(argv[4]);
    float y_offset = atof(argv[5]);
    float distance = atof(argv[6]);
    
    // 默认参数
    float phase_deg = (argc > 7) ? atof(argv[7]) : 0.0f;    // 默认相位0°
    int point_count = (argc > 8) ? atoi(argv[8]) : 100;     // 默认100个采样点
    int delay_ms = (argc > 9) ? atoi(argv[9]) : 25;         // 默认25ms延时

    // 参数验证
    if (amplitude <= 0 || frequency <= 0 || distance <= 0 || x_end <= x_start)
    {
        LOG_D("❌ Error: Invalid parameters\n");
        LOG_D("   amplitude, frequency, distance must be positive\n");
        LOG_D("   x_end must be greater than x_start\n");
        return;
    }
    
    if (point_count < 10 || point_count > 500)
    {
        LOG_D("❌ Error: Point count must be between 10 and 500\n");
        return;
    }
    
    if (delay_ms < 1 || delay_ms > 2000)
    {
        LOG_D("❌ Error: Delay must be between 1ms and 2000ms\n");
        return;
    }

    float phase_rad = phase_deg * M_PI / 180.0f; // 转换为弧度

    laser_draw_sine_wave(amplitude, frequency, phase_rad, x_start, x_end, y_offset, distance, point_count, delay_ms);
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_draw_sine, laser_draw_sine, Draw sine wave with laser points);

/**
 * @brief 激光绘制组合图形命令 (演示模式)
 */
static void cmd_laser_draw_demo(int argc, char **argv)
{
    float distance = 5.0f; // 默认5米距离
    
    if (argc == 2)
    {
        distance = atof(argv[1]);
        if (distance <= 0)
        {
            LOG_D("❌ Error: Distance must be positive\n");
            return;
        }
    }
    else if (argc > 2)
    {
        LOG_D("Usage: laser_draw_demo [distance]\n");
        LOG_D("Example: laser_draw_demo 6.0\n");
        LOG_D("  distance: Drawing distance from gimbal (m, default=5.0)\n");
        return;
    }

    LOG_D("🎨 Starting laser drawing demonstration at %.1fm distance...\n", distance);
    
    // 回到home点
    LOG_D("\n🏠 Returning to home position...\n");
    laser_gimbal_home(&g_laser_gimbal);
    rt_thread_mdelay(2000);
    
    // 绘制矩形
    LOG_D("\n📐 Demo 1: Drawing rectangle...\n");
    laser_draw_rectangle(2.0f, 1.0f, 2.5f, 1.5f, distance, 15, 60);
    rt_thread_mdelay(1000);
    
    // 绘制圆形
    LOG_D("\n⭕ Demo 2: Drawing circle...\n");
    laser_draw_circle(1.0f, -1.0f, 1.0f, distance, 40, 40);
    rt_thread_mdelay(1000);
    
    // 绘制正弦波
    LOG_D("\n〰️ Demo 3: Drawing sine wave...\n");
    laser_draw_sine_wave(0.8f, 2.0f, 0.0f, -1.0f, 3.0f, 2.5f, distance, 80, 30);
    rt_thread_mdelay(1000);
    
    // 绘制带相位的正弦波
    LOG_D("\n〰️ Demo 4: Drawing phase-shifted sine wave...\n");
    laser_draw_sine_wave(0.6f, 1.5f, M_PI/4, -1.5f, 2.5f, -1.5f, distance, 60, 35);
    
    // 回到home点
    LOG_D("\n🏠 Returning to home position...\n");
    laser_gimbal_home(&g_laser_gimbal);
    
    LOG_D("\n✅ Drawing demonstration completed!\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_draw_demo, laser_draw_demo, Run laser drawing demonstration);

/**
 * @brief 显示激光绘制参数默认值命令
 */
static void cmd_laser_draw_defaults(int argc, char **argv)
{
    LOG_D("📋 === Laser Drawing Default Parameters ===\n\n");
    
    LOG_D("🔳 Rectangle Drawing (laser_draw_rect):\n");
    LOG_D("  • Default points per edge: 20\n");
    LOG_D("  • Default delay per point: 50ms\n");
    LOG_D("  • Range: points [2-100], delay [10-2000ms]\n\n");
    
    LOG_D("⭕ Circle Drawing (laser_draw_circle):\n");
    LOG_D("  • Default points on circumference: 60\n");
    LOG_D("  • Default delay per point: 30ms\n");
    LOG_D("  • Range: points [8-200], delay [10-2000ms]\n\n");
    
    LOG_D("〰️ Sine Wave Drawing (laser_draw_sine):\n");
    LOG_D("  • Default phase offset: 0° (degrees)\n");
    LOG_D("  • Default sample points: 100\n");
    LOG_D("  • Default delay per point: 25ms\n");
    LOG_D("  • Range: points [10-500], delay [5-2000ms]\n\n");
    
    LOG_D("💡 Usage Tips:\n");
    LOG_D("  • Lower delay = faster drawing, higher delay = smoother motion\n");
    LOG_D("  • More points = smoother curves, fewer points = faster completion\n");
    LOG_D("  • Adjust parameters based on your application needs\n\n");
    
    LOG_D("📝 Example Commands:\n");
    LOG_D("  laser_draw_rect 2.0 1.0 3.0 2.0 5.0         # Use defaults (20 points, 50ms)\n");
    LOG_D("  laser_draw_rect 2.0 1.0 3.0 2.0 5.0 30 100  # Custom (30 points, 100ms)\n");
    LOG_D("  laser_draw_circle 1.0 1.0 1.5 5.0           # Use defaults (60 points, 30ms)\n");
    LOG_D("  laser_draw_circle 1.0 1.0 1.5 5.0 80 20     # Custom (80 points, 20ms)\n");
    LOG_D("  laser_draw_sine 1.0 2.0 0.0 4.0 2.0 5.0     # Use defaults (0°, 100 points, 25ms)\n");
    LOG_D("  laser_draw_sine 1.0 2.0 0.0 4.0 2.0 5.0 90.0 150 15  # Custom phase, points, delay\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_draw_defaults, laser_draw_defaults, Show laser drawing default parameters and usage tips);

/**
 * @brief 显示角度控制帮助信息
 */
static void cmd_laser_angle_help(int argc, char **argv)
{
    LOG_D("📐 === Laser Gimbal Angle Control Commands ===\n\n");
    
    LOG_D("🎯 Basic Angle Control:\n");
    LOG_D("  laser_angle <yaw> <pitch>     - Set both yaw and pitch angles\n");
    LOG_D("  laser_yaw <yaw>               - Set yaw only (pitch unchanged)\n");
    LOG_D("  laser_pitch <pitch>           - Set pitch only (yaw unchanged)\n");
    LOG_D("  laser_home                    - Return to home position (0°, 0°)\n\n");
    
    LOG_D("🔄 Relative Control:\n");
    LOG_D("  laser_adjust <yaw_Δ> <pitch_Δ> - Adjust angles relatively\n");
    LOG_D("    Example: laser_adjust 10.0 -5.0  (yaw +10°, pitch -5°)\n\n");
    
    LOG_D("⚡ Speed Control:\n");
    LOG_D("  laser_smooth <yaw> <pitch> [speed] - Move with speed control\n");
    LOG_D("    speed: 0.1-2.0 (0.1=slow, 1.0=normal, 2.0=fast)\n");
    LOG_D("    Example: laser_smooth 30.0 15.0 0.5  (half speed)\n\n");
    
    LOG_D("� Velocity Control:\n");
    LOG_D("  laser_yaw_speed <speed>       - Set yaw angular velocity (°/s)\n");
    LOG_D("    Example: laser_yaw_speed 15.0  (yaw at 15°/s)\n");
    LOG_D("  laser_pitch_speed <speed>     - Set pitch angular velocity (°/s)\n");
    LOG_D("    Example: laser_pitch_speed -10.0  (pitch at -10°/s)\n");
    LOG_D("  laser_angular_velocity <yaw_speed> <pitch_speed> - Set both velocities\n");
    LOG_D("    Example: laser_angular_velocity 20.0 -15.0\n");
    LOG_D("  laser_get_speed               - Get current angular velocities\n");
    LOG_D("  laser_stop_velocity           - Stop velocity control mode\n\n");
    
    LOG_D("🔄 Gyro Compensation:\n");
    LOG_D("  laser_gyro_enable <0|1>       - Enable/disable gyro compensation\n");
    LOG_D("  laser_gyro_gain <yaw> <pitch> - Set compensation gains\n");
    LOG_D("  laser_gyro_calibrate [samples] - Start gyro calibration\n");
    LOG_D("  laser_gyro_status             - Show gyro compensation status\n");
    LOG_D("  laser_gyro_help               - Show detailed gyro commands help\n\n");
    
    LOG_D("📊 Status & Information:\n");
    
    LOG_D("�📊 Status & Information:\n");
    LOG_D("  laser_get_angle               - Get current yaw and pitch\n");
    LOG_D("  laser_status                  - Show detailed gimbal status\n");
    LOG_D("  laser_stop                    - Stop gimbal motion immediately\n\n");
    
    LOG_D("🧪 Test & Demo:\n");
    LOG_D("  laser_test angle              - Run angle control demonstration\n");
    LOG_D("  laser_test basic              - Run basic functionality test\n");
    LOG_D("  laser_test scan               - Run scanning pattern test\n\n");
    
    LOG_D("📝 Angle Range & Conventions:\n");
    LOG_D("  • Yaw (偏航): -180° to +180° (left/right rotation)\n");
    LOG_D("    Positive: Turn right, Negative: Turn left\n");
    LOG_D("  • Pitch (俯仰): -180° to +180° (up/down rotation)\n");
    LOG_D("    Positive: Turn up, Negative: Turn down\n");
    LOG_D("  • Home position: (0°, 0°) - Forward direction\n\n");
    
    LOG_D("⚡ Velocity Range & Limits:\n");
    LOG_D("  • Angular velocity: ±60.0°/s maximum\n");
    LOG_D("  • Positive velocity: Same direction as positive angles\n");
    LOG_D("  • Negative velocity: Opposite direction\n");
    LOG_D("  • Velocity mode: Continuous motion until stopped\n\n");
    
    LOG_D("💡 Usage Tips:\n");
    LOG_D("  • Start with laser_init to initialize the system\n");
    LOG_D("  • Use laser_get_angle to check current position\n");
    LOG_D("  • Use laser_adjust for small incremental moves\n");
    LOG_D("  • Use laser_smooth for controlled speed movements\n");
    LOG_D("  • Use velocity commands for continuous motion control\n");
    LOG_D("  • Use laser_stop or laser_stop_velocity to stop motion\n\n");
    
    LOG_D("🚀 Quick Start Examples:\n");
    LOG_D("  # Position Control:\n");
    LOG_D("  laser_init                    # Initialize system\n");
    LOG_D("  laser_angle 0 0               # Go to home position\n");
    LOG_D("  laser_yaw 30                  # Turn right 30°\n");
    LOG_D("  laser_pitch 15                # Tilt up 15°\n");
    LOG_D("  laser_adjust -10 0            # Turn left 10° more\n");
    LOG_D("  laser_smooth 0 0 0.3          # Slowly return home\n\n");
    
    LOG_D("  # Velocity Control:\n");
    LOG_D("  laser_yaw_speed 20.0          # Rotate right at 20°/s\n");
    LOG_D("  laser_pitch_speed -15.0       # Tilt down at 15°/s\n");
    LOG_D("  laser_angular_velocity 10 5   # Yaw 10°/s, pitch 5°/s\n");
    LOG_D("  laser_get_speed               # Check current velocities\n");
    LOG_D("  laser_stop_velocity           # Stop velocity mode\n\n");
    
    LOG_D("  laser_get_angle               # Check final position\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_angle_help, laser_angle_help, Show angle control commands help and usage guide);

/* ======================== 陀螺仪补偿测试命令 ======================== */

/**
 * @brief 启用/禁用陀螺仪补偿
 */
static void cmd_laser_gyro_enable(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: laser_gyro_enable <0|1>");
        LOG_D("  0: Disable gyro compensation");
        LOG_D("  1: Enable gyro compensation");
        return;
    }

    bool enable = (atoi(argv[1]) != 0);
    rt_err_t result = laser_gimbal_enable_gyro_compensation(&g_laser_gimbal, enable);
    
    if (result == RT_EOK)
    {
        LOG_I("✅ Gyro compensation %s", enable ? "ENABLED" : "DISABLED");
    }
    else
    {
        LOG_E("❌ Failed to %s gyro compensation", enable ? "enable" : "disable");
    }
}
MSH_CMD_EXPORT(cmd_laser_gyro_enable, Enable/disable gyro compensation: laser_gyro_enable <0|1>);

/**
 * @brief 设置陀螺仪补偿增益
 */
static void cmd_laser_gyro_gain(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: laser_gyro_gain <yaw_gain> <pitch_gain>");
        LOG_D("  yaw_gain: Yaw axis compensation gain (0.0-2.0)");
        LOG_D("  pitch_gain: Pitch axis compensation gain (0.0-2.0)");
        LOG_D("  Example: laser_gyro_gain 0.5 0.3");
        return;
    }

    float yaw_gain = atof(argv[1]);
    float pitch_gain = atof(argv[2]);
    
    rt_err_t result = laser_gimbal_set_gyro_compensation_gain(&g_laser_gimbal, yaw_gain, pitch_gain);
    
    if (result == RT_EOK)
    {
        LOG_I("✅ Gyro compensation gain set: yaw=%.3f, pitch=%.3f", yaw_gain, pitch_gain);
    }
    else
    {
        LOG_E("❌ Failed to set gyro compensation gain");
    }
}
MSH_CMD_EXPORT(cmd_laser_gyro_gain, Set gyro compensation gain: laser_gyro_gain <yaw_gain> <pitch_gain>);

/**
 * @brief 开始陀螺仪零偏校准
 */
static void cmd_laser_gyro_calibrate(int argc, char **argv)
{
    int samples = 50; // 默认50个采样
    
    if (argc == 2)
    {
        samples = atoi(argv[1]);
    }
    
    if (samples <= 0 || samples > 100)
    {
        LOG_D("Usage: laser_gyro_calibrate [samples]");
        LOG_D("  samples: Number of calibration samples (1-100, default: 50)");
        LOG_D("  Example: laser_gyro_calibrate 80");
        return;
    }

    rt_err_t result = laser_gimbal_start_gyro_calibration(&g_laser_gimbal, samples);
    
    if (result == RT_EOK)
    {
        LOG_I("🔧 Gyro calibration started with %d samples", samples);
        LOG_I("⚠️  Please keep the gimbal STATIONARY during calibration...");
        LOG_I("💡 Use 'laser_gyro_finish_cal' to complete calibration");
    }
    else
    {
        LOG_E("❌ Failed to start gyro calibration");
    }
}
MSH_CMD_EXPORT(cmd_laser_gyro_calibrate, Start gyro calibration: laser_gyro_calibrate [samples]);

/**
 * @brief 完成陀螺仪零偏校准
 */
static void cmd_laser_gyro_finish_cal(int argc, char **argv)
{
    rt_err_t result = laser_gimbal_finish_gyro_calibration(&g_laser_gimbal);
    
    if (result == RT_EOK)
    {
        LOG_I("✅ Gyro calibration completed successfully");
        
        /* 显示校准结果 */
        bool enabled;
        float yaw_gain, pitch_gain, yaw_offset, pitch_offset, roll_offset;
        if (laser_gimbal_get_gyro_compensation_params(&g_laser_gimbal, &enabled,
                                                     &yaw_gain, &pitch_gain,
                                                     &yaw_offset, &pitch_offset, &roll_offset) == RT_EOK)
        {
            LOG_I("📊 Calibration Results:");
            LOG_I("   Yaw offset:   %.3f°/s", yaw_offset);
            LOG_I("   Pitch offset: %.3f°/s", pitch_offset);
            LOG_I("   Roll offset:  %.3f°/s", roll_offset);
        }
    }
    else
    {
        LOG_E("❌ Failed to finish gyro calibration");
    }
}
MSH_CMD_EXPORT(cmd_laser_gyro_finish_cal, Finish gyro calibration and calculate offsets);

/**
 * @brief 手动设置陀螺仪零偏
 */
static void cmd_laser_gyro_offset(int argc, char **argv)
{
    if (argc != 4)
    {
        LOG_D("Usage: laser_gyro_offset <yaw_offset> <pitch_offset> <roll_offset>");
        LOG_D("  yaw_offset: Yaw axis offset in deg/s");
        LOG_D("  pitch_offset: Pitch axis offset in deg/s");
        LOG_D("  roll_offset: Roll axis offset in deg/s");
        LOG_D("  Example: laser_gyro_offset 0.12 -0.05 0.08");
        return;
    }

    float yaw_offset = atof(argv[1]);
    float pitch_offset = atof(argv[2]);
    float roll_offset = atof(argv[3]);
    
    rt_err_t result = laser_gimbal_set_gyro_offset(&g_laser_gimbal, yaw_offset, pitch_offset, roll_offset);
    
    if (result == RT_EOK)
    {
        LOG_I("✅ Gyro offsets set: yaw=%.3f°/s, pitch=%.3f°/s, roll=%.3f°/s", 
              yaw_offset, pitch_offset, roll_offset);
    }
    else
    {
        LOG_E("❌ Failed to set gyro offsets");
    }
}
MSH_CMD_EXPORT(cmd_laser_gyro_offset, Set gyro offsets manually: laser_gyro_offset <yaw> <pitch> <roll>);

/**
 * @brief 显示陀螺仪补偿状态
 */
static void cmd_laser_gyro_status(int argc, char **argv)
{
    bool enabled;
    float yaw_gain, pitch_gain, yaw_offset, pitch_offset, roll_offset;
    
    rt_err_t result = laser_gimbal_get_gyro_compensation_params(&g_laser_gimbal, &enabled,
                                                               &yaw_gain, &pitch_gain,
                                                               &yaw_offset, &pitch_offset, &roll_offset);
    
    if (result != RT_EOK)
    {
        LOG_E("❌ Failed to get gyro compensation parameters");
        return;
    }

    LOG_I("=== Gyro Compensation Status ===");
    LOG_I("Status: %s", enabled ? "ENABLED" : "DISABLED");
    LOG_I("Compensation Gains:");
    LOG_I("  Yaw gain:   %.3f", yaw_gain);
    LOG_I("  Pitch gain: %.3f", pitch_gain);
    LOG_I("Gyro Offsets:");
    LOG_I("  Yaw offset:   %.3f°/s", yaw_offset);
    LOG_I("  Pitch offset: %.3f°/s", pitch_offset);
    LOG_I("  Roll offset:  %.3f°/s", roll_offset);
    
    /* 显示当前陀螺仪数据 */
    extern int hipnuc_get_gyro_data(float *gyro_x, float *gyro_y, float *gyro_z);
    float gyro_x, gyro_y, gyro_z;
    if (hipnuc_get_gyro_data(&gyro_x, &gyro_y, &gyro_z) == 0)
    {
        LOG_I("Current Gyro Data:");
        LOG_I("  X-axis: %.2f°/s", gyro_x);
        LOG_I("  Y-axis: %.2f°/s", gyro_y);  
        LOG_I("  Z-axis: %.2f°/s", gyro_z);
    }
    else
    {
        LOG_W("⚠️  Gyro data not available");
    }
}
MSH_CMD_EXPORT(cmd_laser_gyro_status, Show gyro compensation status and current data);

/**
 * @brief 陀螺仪补偿帮助信息
 */
static void cmd_laser_gyro_help(int argc, char **argv)
{
    LOG_D("🔄 === Laser Gimbal Gyro Compensation Commands ===\n");
    
    LOG_D("🎛️  Basic Control:\n");
    LOG_D("  laser_gyro_enable <0|1>       - Enable/disable gyro compensation\n");
    LOG_D("  laser_gyro_gain <yaw> <pitch> - Set compensation gains (0.0-2.0)\n");
    LOG_D("  laser_gyro_status             - Show current compensation status\n\n");
    
    LOG_D("🔧 Calibration:\n");
    LOG_D("  laser_gyro_calibrate [samples] - Start gyro offset calibration\n");
    LOG_D("  laser_gyro_finish_cal         - Finish calibration and save offsets\n");
    LOG_D("  laser_gyro_offset <y> <p> <r> - Manually set gyro offsets\n\n");
    
    LOG_D("📐 Calibration Process:\n");
    LOG_D("  1. Keep gimbal completely stationary\n");
    LOG_D("  2. Run: laser_gyro_calibrate 80\n");
    LOG_D("  3. Wait for sampling to complete\n");
    LOG_D("  4. Run: laser_gyro_finish_cal\n");
    LOG_D("  5. Enable compensation: laser_gyro_enable 1\n\n");
    
    LOG_D("⚙️  Recommended Settings:\n");
    LOG_D("  • Initial gains: yaw=0.3, pitch=0.3\n");
    LOG_D("  • Calibration samples: 50-100\n");
    LOG_D("  • Keep environment vibration-free during calibration\n");
    LOG_D("  • Test compensation with small movements first\n\n");
    
    LOG_D("🚀 Quick Setup Example:\n");
    LOG_D("  laser_gyro_calibrate 60      # Start calibration with 60 samples\n");
    LOG_D("  laser_gyro_finish_cal        # Complete calibration\n");
    LOG_D("  laser_gyro_gain 0.3 0.3      # Set moderate gains\n");
    LOG_D("  laser_gyro_enable 1          # Enable compensation\n");
    LOG_D("  laser_gyro_status            # Check status\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_laser_gyro_help, laser_gyro_help, Show gyro compensation commands help);
