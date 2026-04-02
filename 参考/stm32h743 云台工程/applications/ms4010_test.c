/**
 * @file ms4010_test.c
 * @brief MS4010电机驱动简单测试文件
 * @version 1.0
 * @date 2025-07-25
 * @author Your Name
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 提供MS4010电机驱动的简单测试功能
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_ms4010.h"

#define DBG_TAG "ms4010.test"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* 测试配置 */
#define MS4010_TEST_RS485_NAME      "uart2"         // RS485设备名称
#define MS4010_TEST_MOTOR_ID        1               // 电机ID
#define MS4010_TEST_BAUDRATE        115200          // 波特率
#define MS4010_TEST_THREAD_STACK    1024            // 线程栈大小
#define MS4010_TEST_THREAD_PRIORITY 15              // 线程优先级

/* 全局变量 */
static ms4010_device_t g_ms4010_dev;               // MS4010设备实例
static rt_thread_t g_test_thread = RT_NULL;        // 测试线程句柄
static rt_bool_t g_test_running = RT_FALSE;        // 测试运行标志

/**
 * @brief 基础通信测试
 */
static rt_err_t ms4010_basic_test(void)
{
    rt_err_t result;
    ms4010_status1_t status1;
    rt_uint32_t send_count, error_count;

    LOG_I("=== MS4010 Basic Communication Test ===");

    /* 测试开环控制 */
    LOG_I("Testing open loop control...");
    result = ms4010_open_loop_control(&g_ms4010_dev, 0);
    if (result != RT_EOK)
    {
        LOG_E("Open loop control test failed: %d", result);
        return result;
    }
    rt_thread_mdelay(100);

    /* 读取状态1 */
    result = ms4010_read_status1(&g_ms4010_dev, &status1);
    if (result == RT_EOK)
    {
        LOG_I("Motor Status1:");
        LOG_I("  Temperature: %d°C", status1.temperature);
        LOG_I("  Voltage: %.2fV", status1.voltage * 0.01f);
        LOG_I("  Current: %.2fA", status1.current * 0.01f);
        LOG_I("  Motor State: 0x%02X", status1.motor_state);
        LOG_I("  Error State: 0x%02X", status1.error_state);
    }
    else
    {
        LOG_W("Failed to get motor status1: %d", result);
    }

    /* 获取统计信息 */
    ms4010_get_statistics(&g_ms4010_dev, &send_count, &error_count);
    LOG_I("Communication Statistics:");
    LOG_I("  Send Count: %d", send_count);
    LOG_I("  Error Count: %d", error_count);

    return RT_EOK;
}

/**
 * @brief 电机控制功能测试
 */
static rt_err_t ms4010_control_test(void)
{
    rt_err_t result;
    ms4010_status1_t status1;

    LOG_I("=== MS4010 Control Function Test ===");

    /* 1. 开环控制测试 */
    LOG_I("1. Open Loop Control Test");
    
    LOG_I("  Setting open loop value to 200...");
    result = ms4010_open_loop_control(&g_ms4010_dev, 200);
    if (result != RT_EOK)
    {
        LOG_E("  Open loop control failed: %d", result);
        return result;
    }
    rt_thread_mdelay(1000);

    /* 读取状态1和状态2 */
    ms4010_status_t status2;
    ms4010_read_status1(&g_ms4010_dev, &status1);
    ms4010_get_status(&g_ms4010_dev, &status2);
    LOG_I("  Motor voltage: %.2fV, current: %.2fA", status1.voltage * 0.01f, status1.current * 0.01f);
    LOG_I("  Current speed: %d dps", status2.speed);

    /* 停止 */
    LOG_I("  Stopping motor...");
    ms4010_motor_stop(&g_ms4010_dev);
    rt_thread_mdelay(500);

    /* 2. 速度控制测试 */
    LOG_I("2. Speed Control Test");
    
    LOG_I("  Setting speed to 360 dps...");
    result = ms4010_speed_control(&g_ms4010_dev, 360);
    if (result != RT_EOK)
    {
        LOG_E("  Speed control failed: %d", result);
        return result;
    }
    rt_thread_mdelay(2000);

    /* 读取状态1和状态2 */
    ms4010_get_status(&g_ms4010_dev, &status2);
    LOG_I("  Current speed: %d dps", status2.speed);

    /* 停止 */
    LOG_I("  Stopping motor...");
    ms4010_motor_stop(&g_ms4010_dev);
    rt_thread_mdelay(500);

    /* 3. 位置控制测试 */
    LOG_I("3. Position Control Test");
    
    LOG_I("  Moving to 90 degrees...");
    result = ms4010_position_control2(&g_ms4010_dev, 9000, 18000); // 90度, 180dps
    if (result != RT_EOK)
    {
        LOG_E("  Position control failed: %d", result);
        return result;
    }
    rt_thread_mdelay(3000);

    /* 读取状态2 */
    ms4010_get_status(&g_ms4010_dev, &status2);
    LOG_I("  Current encoder: %u", status2.encoder);

    /* 回到原点 */
    LOG_I("  Returning to origin...");
    ms4010_position_control2(&g_ms4010_dev, 0, 18000);
    rt_thread_mdelay(3000);

    LOG_I("Control function test completed!");
    return RT_EOK;
}

/**
 * @brief 连续运行测试
 */
static void ms4010_continuous_test_thread(void *parameter)
{
    rt_err_t result;
    ms4010_status1_t status1;
    ms4010_status_t status2;
    rt_uint32_t loop_count = 0;
    rt_int32_t speed_targets[] = {0, 180, 0, -180, 0}; // 速度序列 (dps)
    rt_uint8_t speed_index = 0;

    LOG_I("=== MS4010 Continuous Test Started ===");

    while (g_test_running)
    {
        loop_count++;
        
        /* 每10次循环改变一次速度 */
        if (loop_count % 10 == 0)
        {
            rt_int32_t target_speed = speed_targets[speed_index];
            LOG_I("Loop %d: Setting speed to %d dps", loop_count, target_speed);
            
            result = ms4010_speed_control(&g_ms4010_dev, target_speed);
            if (result != RT_EOK)
            {
                LOG_E("Speed control failed at loop %d: %d", loop_count, result);
            }
            
            speed_index = (speed_index + 1) % (sizeof(speed_targets) / sizeof(speed_targets[0]));
        }

        /* 读取状态1和状态2 */
        result = ms4010_read_status1(&g_ms4010_dev, &status1);
        rt_err_t result2 = ms4010_get_status(&g_ms4010_dev, &status2);
        if (result == RT_EOK && result2 == RT_EOK)
        {
            LOG_D("Loop %d: Temp=%d°C, Current=%.2fA, Speed=%d dps, Encoder=%u", 
                  loop_count, status1.temperature, status1.current * 0.01f, status2.speed, status2.encoder);
        }
        else
        {
            LOG_W("Failed to get status at loop %d", loop_count);
        }

        /* 每50次循环显示统计信息 */
        if (loop_count % 50 == 0)
        {
            rt_uint32_t send_count, error_count;
            ms4010_get_statistics(&g_ms4010_dev, &send_count, &error_count);
            LOG_I("Statistics at loop %d: Send=%d, Error=%d, Success Rate=%.1f%%", 
                  loop_count, send_count, error_count, 
                  send_count > 0 ? (100.0 * (send_count - error_count) / send_count) : 0.0);
        }

        rt_thread_mdelay(100); // 100ms周期
    }

    /* 停止电机 */
    ms4010_speed_control(&g_ms4010_dev, 0);
    LOG_I("=== MS4010 Continuous Test Stopped ===");
}

/**
 * @brief 初始化MS4010测试
 */
static int ms4010_test_init(void)
{
    rt_err_t result;

    LOG_I("Initializing MS4010 test...");

    /* 初始化MS4010设备 */
    result = ms4010_init(&g_ms4010_dev, MS4010_TEST_RS485_NAME, MS4010_TEST_MOTOR_ID, MS4010_TEST_BAUDRATE);
    if (result != RT_EOK)
    {
        LOG_E("Failed to initialize MS4010 device: %d", result);
        return result;
    }

    LOG_I("MS4010 test initialized successfully");
    return RT_EOK;
}
#define RT_USING_FINSH
/* MSH命令实现 */
#ifdef RT_USING_FINSH

static void ms4010_test(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ms4010_test <command>\n");
        rt_kprintf("Commands:\n");
        rt_kprintf("  init           - Initialize MS4010 device\n");
        rt_kprintf("  deinit         - Deinitialize MS4010 device\n");
        rt_kprintf("  basic          - Run basic communication test\n");
        rt_kprintf("  control        - Run control function test\n");
        rt_kprintf("  start          - Start continuous test\n");
        rt_kprintf("  stop           - Stop continuous test\n");
        rt_kprintf("  status         - Show current motor status\n");
        rt_kprintf("  stats          - Show communication statistics\n");
        rt_kprintf("  open <value>   - Open loop control (-1000~1000)\n");
        rt_kprintf("  move <speed>   - Set motor speed (dps)\n");
        rt_kprintf("  pos <angle> [speed] - Move to position (0.01°, opt speed)\n");
        rt_kprintf("  pos2 <angle> <speed> - Move to position2 (0.01°, with speed limit)\n");
        rt_kprintf("  single1 <dir> <angle> - Single turn pos control1 (no speed limit)\n");
        rt_kprintf("  single2 <dir> <angle> [speed] - Single turn pos control2 (with speed)\n");
        rt_kprintf("  increment1 <angle> - Increment pos control1 (no speed limit)\n");
        rt_kprintf("  increment2 <angle> [speed] - Increment pos control2 (with speed)\n");
        rt_kprintf("  torque <value> - Torque control\n");
        rt_kprintf("  motor_on       - Turn motor on\n");
        rt_kprintf("  motor_off      - Turn motor off\n");
        rt_kprintf("  motor_stop     - Stop motor immediately\n");
        rt_kprintf("  brake <cmd>    - Brake control (0=lock, 1=release, 16=read)\n");
        rt_kprintf("  clear_error    - Clear motor error flags\n");
        rt_kprintf("  set_pos <angle> - Set current position to specified angle (0.01°)\n");
        return;
    }

    if (rt_strcmp(argv[1], "init") == 0)
    {
        rt_err_t result = ms4010_test_init();
        rt_kprintf("MS4010 initialization %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "deinit") == 0)
    {
        rt_err_t result = ms4010_deinit(&g_ms4010_dev);
        rt_kprintf("MS4010 deinitialization %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "basic") == 0)
    {
        rt_err_t result = ms4010_basic_test();
        rt_kprintf("Basic test %s\n", result == RT_EOK ? "PASSED" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "control") == 0)
    {
        rt_err_t result = ms4010_control_test();
        rt_kprintf("Control test %s\n", result == RT_EOK ? "PASSED" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "start") == 0)
    {
        if (g_test_running)
        {
            rt_kprintf("Continuous test is already running\n");
            return;
        }

        g_test_running = RT_TRUE;
        g_test_thread = rt_thread_create("ms4010_test", ms4010_continuous_test_thread, RT_NULL, 
                                        MS4010_TEST_THREAD_STACK, MS4010_TEST_THREAD_PRIORITY, 20);
        if (g_test_thread != RT_NULL)
        {
            rt_thread_startup(g_test_thread);
            rt_kprintf("Continuous test started\n");
        }
        else
        {
            g_test_running = RT_FALSE;
            rt_kprintf("Failed to create test thread\n");
        }
    }
    else if (rt_strcmp(argv[1], "stop") == 0)
    {
        if (!g_test_running)
        {
            rt_kprintf("No continuous test is running\n");
            return;
        }

        g_test_running = RT_FALSE;
        if (g_test_thread != RT_NULL)
        {
            rt_thread_delete(g_test_thread);
            g_test_thread = RT_NULL;
        }
        rt_kprintf("Continuous test stopped\n");
    }
    else if (rt_strcmp(argv[1], "status") == 0)
    {
        ms4010_status1_t status1;
        ms4010_status_t status2;
        ms4010_status3_t status3;
        rt_err_t result1 = ms4010_read_status1(&g_ms4010_dev, &status1);
        rt_err_t result2 = ms4010_get_status(&g_ms4010_dev, &status2);
        rt_err_t result3 = ms4010_read_status3(&g_ms4010_dev, &status3);
        
        if (result1 == RT_EOK)
        {
            rt_kprintf("Motor Status1:\n");
            rt_kprintf("  Temperature: %d°C\n", status1.temperature);
            rt_kprintf("  Voltage: %.2fV\n", status1.voltage * 0.01f);
            rt_kprintf("  Current: %.2fA\n", status1.current * 0.01f);
            rt_kprintf("  Motor State: 0x%02X\n", status1.motor_state);
            rt_kprintf("  Error State: 0x%02X\n", status1.error_state);
        }
        else
        {
            rt_kprintf("Failed to get motor status1: %d\n", result1);
        }
        
        if (result2 == RT_EOK)
        {
            rt_kprintf("Motor Status2:\n");
            rt_kprintf("  Temperature: %d°C\n", status2.temperature);
            rt_kprintf("  Power/Torque: %d\n", status2.power_or_torque);
            rt_kprintf("  Speed: %d dps\n", status2.speed);
            rt_kprintf("  Encoder: %u\n", status2.encoder);
        }
        else
        {
            rt_kprintf("Failed to get motor status2: %d\n", result2);
        }
        
        if (result3 == RT_EOK)
        {
            rt_kprintf("Motor Status3:\n");
            rt_kprintf("  Temperature: %d°C\n", status3.temperature);
            rt_kprintf("  Current A: %d\n", status3.current_a);
            rt_kprintf("  Current B: %d\n", status3.current_b);
            rt_kprintf("  Current C: %d\n", status3.current_c);
        }
        else
        {
            rt_kprintf("Failed to get motor status3: %d\n", result3);
        }
    }
    else if (rt_strcmp(argv[1], "stats") == 0)
    {
        rt_uint32_t send_count, error_count;
        ms4010_get_statistics(&g_ms4010_dev, &send_count, &error_count);
        rt_kprintf("Communication Statistics:\n");
        rt_kprintf("  Total Send: %d\n", send_count);
        rt_kprintf("  Total Error: %d\n", error_count);
        rt_kprintf("  Success Rate: %.1f%%\n", 
                   send_count > 0 ? (100.0 * (send_count - error_count) / send_count) : 0.0);
    }
    else if (rt_strcmp(argv[1], "open") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test open <value>\n");
            rt_kprintf("  value: open loop control value (-1000~1000)\n");
            return;
        }

        rt_int16_t value = atoi(argv[2]);
        rt_err_t result = ms4010_open_loop_control(&g_ms4010_dev, value);
        rt_kprintf("Open loop control to %d: %s\n", value, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "move") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test move <speed>\n");
            return;
        }

        rt_int32_t speed = atoi(argv[2]);
        rt_err_t result = ms4010_speed_control(&g_ms4010_dev, speed);
        rt_kprintf("Speed control to %d dps: %s\n", speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "pos") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test pos <angle> [speed]\n");
            rt_kprintf("  angle: target angle in 0.01 degree units\n");
            rt_kprintf("  speed: optional speed (default 18000)\n");
            return;
        }

        rt_int32_t angle = atoi(argv[2]);
        rt_uint16_t speed = (argc >= 4) ? atoi(argv[3]) : 18000;
        rt_err_t result = ms4010_position_control(&g_ms4010_dev, angle, speed);
        rt_kprintf("Position control to %d (0.01°) @ speed %d: %s\n", angle, speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "pos2") == 0)
    {
        if (argc < 4)
        {
            rt_kprintf("Usage: ms4010_test pos2 <angle> <max_speed>\n");
            rt_kprintf("  angle: target angle in 0.01 degree units\n"); 
            rt_kprintf("  max_speed: maximum speed in 0.01dps units\n");
            return;
        }

        rt_int64_t angle = atoll(argv[2]);
        rt_uint32_t max_speed = atoi(argv[3]);
        rt_err_t result = ms4010_position_control2(&g_ms4010_dev, angle, max_speed);
        rt_kprintf("Position control2 to %lld (0.01°) @ max_speed %u: %s\n", angle, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "single1") == 0)
    {
        if (argc < 4)
        {
            rt_kprintf("Usage: ms4010_test single1 <direction> <angle>\n");
            rt_kprintf("  direction: 0=clockwise, 1=counter-clockwise\n");
            rt_kprintf("  angle: target angle in 0.01 degree units (0-35999)\n");
            return;
        }

        rt_uint8_t direction = atoi(argv[2]);
        rt_uint16_t angle = atoi(argv[3]);
        rt_err_t result = ms4010_single_position_control1(&g_ms4010_dev, direction, angle);
        rt_kprintf("Single position control1 dir=%d, angle=%d: %s\n", direction, angle, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "single2") == 0)
    {
        if (argc < 4)
        {
            rt_kprintf("Usage: ms4010_test single2 <direction> <angle> [max_speed]\n");
            rt_kprintf("  direction: 0=clockwise, 1=counter-clockwise\n");
            rt_kprintf("  angle: target angle in 0.01 degree units (0-35999)\n");
            rt_kprintf("  max_speed: optional max speed in 0.01dps units (default 18000)\n");
            return;
        }

        rt_uint8_t direction = atoi(argv[2]);
        rt_uint16_t angle = atoi(argv[3]);
        rt_uint32_t max_speed = (argc >= 5) ? atoi(argv[4]) : 18000;
        rt_err_t result = ms4010_single_position_control2(&g_ms4010_dev, direction, angle, max_speed);
        rt_kprintf("Single position control2 dir=%d, angle=%d @ max_speed %u: %s\n", direction, angle, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "increment1") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test increment1 <angle>\n");
            rt_kprintf("  angle: angle increment in 0.01 degree units\n");
            return;
        }

        rt_int32_t angle_increment = atoi(argv[2]);
        rt_err_t result = ms4010_increment_position_control1(&g_ms4010_dev, angle_increment);
        rt_kprintf("Increment position control1 %d (0.01°): %s\n", angle_increment, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "increment2") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test increment2 <angle> [max_speed]\n");
            rt_kprintf("  angle: angle increment in 0.01 degree units\n");
            rt_kprintf("  max_speed: optional max speed in 0.01dps units (default 18000)\n");
            return;
        }

        rt_int32_t angle_increment = atoi(argv[2]);
        rt_uint32_t max_speed = (argc >= 4) ? atoi(argv[3]) : 18000;
        rt_err_t result = ms4010_increment_position_control2(&g_ms4010_dev, angle_increment, max_speed);
        rt_kprintf("Increment position control2 %d (0.01°) @ max_speed %u: %s\n", angle_increment, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "torque") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test torque <value>\n");
            return;
        }

        rt_int16_t torque = atoi(argv[2]);
        rt_err_t result = ms4010_torque_control(&g_ms4010_dev, torque);
        rt_kprintf("Torque control to %d: %s\n", torque, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "motor_on") == 0)
    {
        rt_err_t result = ms4010_motor_on(&g_ms4010_dev);
        rt_kprintf("Motor on: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "motor_off") == 0)
    {
        rt_err_t result = ms4010_motor_off(&g_ms4010_dev);
        rt_kprintf("Motor off: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "motor_stop") == 0)
    {
        rt_err_t result = ms4010_motor_stop(&g_ms4010_dev);
        rt_kprintf("Motor stop: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "brake") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test brake <cmd>\n");
            rt_kprintf("  cmd: 0=lock, 1=release, 16=read status\n");
            return;
        }

        rt_uint8_t brake_cmd = atoi(argv[2]);
        rt_uint8_t brake_status = 0;
        rt_err_t result = ms4010_brake_control(&g_ms4010_dev, brake_cmd, &brake_status);
        if (brake_cmd == MS4010_BRAKE_READ_STATUS)
        {
            rt_kprintf("Brake status read: %s, status=0x%02X\n", result == RT_EOK ? "SUCCESS" : "FAILED", brake_status);
        }
        else
        {
            rt_kprintf("Brake control cmd=%d: %s\n", brake_cmd, result == RT_EOK ? "SUCCESS" : "FAILED");
        }
    }
    else if (rt_strcmp(argv[1], "clear_error") == 0)
    {
        rt_err_t result = ms4010_clear_error(&g_ms4010_dev);
        rt_kprintf("Clear error: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "set_pos") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ms4010_test set_pos <angle>\n");
            rt_kprintf("  angle: angle value in 0.01 degree units\n");
            return;
        }

        rt_int32_t angle = atoi(argv[2]);
        rt_err_t result = ms4010_set_current_position(&g_ms4010_dev, angle);
        rt_kprintf("Set current position to %d (0.01°): %s\n", angle, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else
    {
        rt_kprintf("Unknown command: %s\n", argv[1]);
    }
}
MSH_CMD_EXPORT(ms4010_test, MS4010 motor driver test commands);

#endif /* RT_USING_FINSH */

/* 自动初始化 - 可以注释掉这行来禁用自动初始化 */
// INIT_APP_EXPORT(ms4010_test_init);
