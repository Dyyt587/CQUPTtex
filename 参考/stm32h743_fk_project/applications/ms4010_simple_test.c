/**
 * @file ms4010_simple_test.c
 * @brief MS4010电机驱动最简单测试
 * @version 1.0
 * @date 2025-07-25
 * @author Your Name
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 提供MS4010电机驱动的最简单测试，用于快速验证驱动是否正常工作
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_ms4010.h"

#define DBG_TAG "ms4010.simple"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* 简单测试配置 */
#define SIMPLE_TEST_RS485_NAME      "uart2"     // 根据你的实际配置修改
#define SIMPLE_TEST_MOTOR_ID        1           // 电机ID
#define SIMPLE_TEST_BAUDRATE        115200      // 波特率
//#define SIMPLE_TEST_BAUDRATE        1000000      // 波特率

/* 全局设备实例 */
static ms4010_device_t simple_motor;

/**
 * @brief 简单的电机测试线程
 */
static void simple_test_thread(void *parameter)
{
    rt_err_t result;
    ms4010_status1_t status1;
    ms4010_status_t status2;
    ms4010_status3_t status3;
    rt_uint32_t test_count = 0;

    LOG_I("Starting MS4010 simple test...");

    /* 初始化电机 */
    result = ms4010_init(&simple_motor, SIMPLE_TEST_RS485_NAME, SIMPLE_TEST_MOTOR_ID, SIMPLE_TEST_BAUDRATE);
    if (result != RT_EOK)
    {
        LOG_E("Failed to initialize MS4010: %d", result);
        return;
    }

    LOG_I("MS4010 initialized successfully!");

    /* 简单的测试循环 */
    while (1)
    {
        test_count++;
        
        LOG_I("=== Test Loop %d ===", test_count);

        /* 1. 测试开环控制 - 正转 */
        LOG_I("Step 1: Forward rotation (200)");
        result = ms4010_open_loop_control(&simple_motor, 200);
        if (result == RT_EOK)
        {
            LOG_I("  Command sent successfully");
        }
        else
        {
            LOG_E("  Command failed: %d", result);
        }
        
        /* 等待2秒 */
        rt_thread_mdelay(2000);

        /* 读取状态1和状态2 */
        result = ms4010_read_status1(&simple_motor, &status1);
        rt_err_t result2 = ms4010_get_status(&simple_motor, &status2);
        if (result == RT_EOK && result2 == RT_EOK)
        {
            LOG_I("  Status1: Temp=%d°C, Voltage=%.2fV, Current=%.2fA", 
                  status1.temperature, status1.voltage * 0.01f, status1.current * 0.01f);
            LOG_I("  Status2: Speed=%d dps, Encoder=%u", status2.speed, status2.encoder);
        }
        else
        {
            LOG_W("  Failed to read status: %d, %d", result, result2);
        }

        /* 2. 停止电机 */
        LOG_I("Step 2: Stop motor");
        ms4010_motor_stop(&simple_motor);
        rt_thread_mdelay(1000);

        /* 3. 测试开环控制 - 反转 */
        LOG_I("Step 3: Reverse rotation (-200)");
        ms4010_open_loop_control(&simple_motor, -200);
        rt_thread_mdelay(2000);

        /* 读取状态3 (相电流状态) */
        result = ms4010_read_status3(&simple_motor, &status3);
        if (result == RT_EOK)
        {
            LOG_I("  Status3: Temp=%d°C, Current_A=%d, Current_B=%d, Current_C=%d", 
                  status3.temperature, status3.current_a, status3.current_b, status3.current_c);
        }
        else
        {
            LOG_W("  Failed to read status3: %d", result);
        }

        /* 4. 停止电机 */
        LOG_I("Step 4: Stop motor");
        ms4010_motor_stop(&simple_motor);

        /* 显示统计信息 */
        rt_uint32_t send_count, error_count;
        ms4010_get_statistics(&simple_motor, &send_count, &error_count);
        LOG_I("Statistics: Send=%u, Error=%u, Success Rate=%.1f%%", 
              send_count, error_count,
              send_count > 0 ? ((float)(send_count - error_count) / send_count * 100) : 0);

        /* 等待5秒后进行下一轮测试 */
        LOG_I("Waiting 5 seconds for next test...\n");
        rt_thread_mdelay(5000);
    }
}

/**
 * @brief 启动简单测试
 */
static int ms4010_simple_test_start(void)
{
    rt_thread_t tid;

    LOG_I("Creating MS4010 simple test thread...");

    tid = rt_thread_create("ms4010_simple", 
                          simple_test_thread, 
                          RT_NULL, 
                          1024,     // 栈大小
                          20,       // 优先级
                          20);      // 时间片

    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        LOG_I("MS4010 simple test thread started");
        return RT_EOK;
    }
    else
    {
        LOG_E("Failed to create MS4010 simple test thread");
        return -RT_ERROR;
    }
}

/* MSH 命令支持 */
#ifdef RT_USING_FINSH

static void ms4010_simple(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("MS4010 Simple Test Commands:\n");
        LOG_D("  ms4010_simple start        - Start simple test\n");
        LOG_D("  ms4010_simple init         - Initialize motor only\n");
        LOG_D("  ms4010_simple deinit       - Deinitialize motor\n");
        LOG_D("  ms4010_simple open <value> - Open loop control (-1000~1000)\n");
        LOG_D("  ms4010_simple forward      - Test forward rotation\n");
        LOG_D("  ms4010_simple reverse      - Test reverse rotation\n");
        LOG_D("  ms4010_simple stop         - Stop motor\n");
        LOG_D("  ms4010_simple speed <val>  - Speed control (dps)\n");
        LOG_D("  ms4010_simple torque <val> - Torque control\n");
        LOG_D("  ms4010_simple pos <angle> [speed] - Position control\n");
        LOG_D("  ms4010_simple pos2 <angle> <speed> - Position control2\n");
        LOG_D("  ms4010_simple single1 <dir> <angle> - Single pos control1\n");
        LOG_D("  ms4010_simple single2 <dir> <angle> [speed] - Single pos control2\n");
        LOG_D("  ms4010_simple inc1 <angle> - Increment control1\n");
        LOG_D("  ms4010_simple inc2 <angle> [speed] - Increment control2\n");
        LOG_D("  ms4010_simple status1      - Read motor status1\n");
        LOG_D("  ms4010_simple status2      - Read motor status2\n");
        LOG_D("  ms4010_simple status3      - Read motor status3\n");
        LOG_D("  ms4010_simple stats        - Show communication statistics\n");
        LOG_D("  ms4010_simple motor_on     - Turn motor on\n");
        LOG_D("  ms4010_simple motor_off    - Turn motor off\n");
        LOG_D("  ms4010_simple motor_stop   - Stop motor immediately\n");
        LOG_D("  ms4010_simple brake <cmd>  - Brake control (0=lock,1=release,16=read)\n");
        LOG_D("  ms4010_simple clear_err    - Clear error flags\n");
        LOG_D("  ms4010_simple set_pos <angle> - Set current position (0.01°)\n");
        return;
    }

    if (rt_strcmp(argv[1], "start") == 0)
    {
        ms4010_simple_test_start();
    }
    else if (rt_strcmp(argv[1], "init") == 0)
    {
        rt_err_t result = ms4010_init(&simple_motor, SIMPLE_TEST_RS485_NAME, 
                                     SIMPLE_TEST_MOTOR_ID, SIMPLE_TEST_BAUDRATE);
        LOG_D("MS4010 init: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "deinit") == 0)
    {
        rt_err_t result = ms4010_deinit(&simple_motor);
        LOG_D("MS4010 deinit: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "open") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple open <value>\n");
            LOG_D("  value: open loop control value (-1000~1000)\n");
            return;
        }
        rt_int16_t value = atoi(argv[2]);
        rt_err_t result = ms4010_open_loop_control(&simple_motor, value);
        LOG_D("Open loop control %d: %s\n", value, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "forward") == 0)
    {
        rt_err_t result = ms4010_open_loop_control(&simple_motor, 300);
        LOG_D("Forward rotation: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "reverse") == 0)
    {
        rt_err_t result = ms4010_open_loop_control(&simple_motor, -300);
        LOG_D("Reverse rotation: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "stop") == 0)
    {
        rt_err_t result = ms4010_motor_stop(&simple_motor);
        LOG_D("Stop motor: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "speed") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple speed <value>\n");
            LOG_D("  value: speed in dps\n");
            return;
        }
        rt_int32_t speed = atoi(argv[2]);
        rt_err_t result = ms4010_speed_control(&simple_motor, speed);
        LOG_D("Speed control %d dps: %s\n", speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "torque") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple torque <value>\n");
            return;
        }
        rt_int16_t torque = atoi(argv[2]);
        rt_err_t result = ms4010_torque_control(&simple_motor, torque);
        LOG_D("Torque control %d: %s\n", torque, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "pos") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple pos <angle> [speed]\n");
            LOG_D("  angle: target angle in 0.01 degree units\n");
            LOG_D("  speed: optional speed (default 18000)\n");
            return;
        }
        rt_int32_t angle = atoi(argv[2]);
        rt_uint16_t speed = (argc >= 4) ? atoi(argv[3]) : 18000;
        rt_err_t result = ms4010_position_control(&simple_motor, angle, speed);
        LOG_D("Position control to %d (0.01°) @ speed %d: %s\n", angle, speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "pos2") == 0)
    {
        if (argc < 4)
        {
            LOG_D("Usage: ms4010_simple pos2 <angle> <max_speed>\n");
            LOG_D("  angle: target angle in 0.01 degree units\n");
            LOG_D("  max_speed: max speed in 0.01dps units\n");
            return;
        }
        rt_int64_t angle = atoll(argv[2]);
        rt_uint32_t max_speed = atoi(argv[3]);
        rt_err_t result = ms4010_position_control2(&simple_motor, angle, max_speed);
        LOG_D("Position control2 to %lld (0.01°) @ max_speed %u: %s\n", angle, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "single1") == 0)
    {
        if (argc < 4)
        {
            LOG_D("Usage: ms4010_simple single1 <dir> <angle>\n");
            LOG_D("  dir: 0=clockwise, 1=counter-clockwise\n");
            LOG_D("  angle: target angle in 0.01 degree units (0-35999)\n");
            return;
        }
        rt_uint8_t direction = atoi(argv[2]);
        rt_uint16_t angle = atoi(argv[3]);
        rt_err_t result = ms4010_single_position_control1(&simple_motor, direction, angle);
        LOG_D("Single position control1 dir=%d, angle=%d: %s\n", direction, angle, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "single2") == 0)
    {
        if (argc < 4)
        {
            LOG_D("Usage: ms4010_simple single2 <dir> <angle> [max_speed]\n");
            LOG_D("  dir: 0=clockwise, 1=counter-clockwise\n");
            LOG_D("  angle: target angle in 0.01 degree units (0-35999)\n");
            LOG_D("  max_speed: optional max speed (default 18000)\n");
            return;
        }
        rt_uint8_t direction = atoi(argv[2]);
        rt_uint16_t angle = atoi(argv[3]);
        rt_uint32_t max_speed = (argc >= 5) ? atoi(argv[4]) : 18000;
        rt_err_t result = ms4010_single_position_control2(&simple_motor, direction, angle, max_speed);
        LOG_D("Single position control2 dir=%d, angle=%d @ max_speed %u: %s\n", direction, angle, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "inc1") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple inc1 <angle>\n");
            LOG_D("  angle: angle increment in 0.01 degree units\n");
            return;
        }
        rt_int32_t angle_increment = atoi(argv[2]);
        rt_err_t result = ms4010_increment_position_control1(&simple_motor, angle_increment);
        LOG_D("Increment position control1 %d (0.01°): %s\n", angle_increment, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "inc2") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple inc2 <angle> [max_speed]\n");
            LOG_D("  angle: angle increment in 0.01 degree units\n");
            LOG_D("  max_speed: optional max speed (default 18000)\n");
            return;
        }
        rt_int32_t angle_increment = atoi(argv[2]);
        rt_uint32_t max_speed = (argc >= 4) ? atoi(argv[3]) : 18000;
        rt_err_t result = ms4010_increment_position_control2(&simple_motor, angle_increment, max_speed);
        LOG_D("Increment position control2 %d (0.01°) @ max_speed %u: %s\n", angle_increment, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "status1") == 0)
    {
        ms4010_status1_t status1;
        rt_err_t result = ms4010_read_status1(&simple_motor, &status1);
        if (result == RT_EOK)
        {
            LOG_D("Motor Status1:\n");
            LOG_D("  Temperature: %d°C\n", status1.temperature);
            LOG_D("  Voltage: %.2fV\n", status1.voltage * 0.01f);
            LOG_D("  Current: %.2fA\n", status1.current * 0.01f);
            LOG_D("  Motor State: 0x%02X\n", status1.motor_state);
            LOG_D("  Error State: 0x%02X\n", status1.error_state);
        }
        else
        {
            LOG_D("Failed to read status1: %d\n", result);
        }
    }
    else if (rt_strcmp(argv[1], "status2") == 0)
    {
        ms4010_status_t status2;
        rt_err_t result = ms4010_get_status(&simple_motor, &status2);
        if (result == RT_EOK)
        {
            LOG_D("Motor Status2:\n");
            LOG_D("  Temperature: %d°C\n", status2.temperature);
            LOG_D("  Power/Torque: %d\n", status2.power_or_torque);
            LOG_D("  Speed: %d dps\n", status2.speed);
            LOG_D("  Encoder: %u\n", status2.encoder);
        }
        else
        {
            LOG_D("Failed to read status2: %d\n", result);
        }
    }
    else if (rt_strcmp(argv[1], "status3") == 0)
    {
        ms4010_status3_t status3;
        rt_err_t result = ms4010_read_status3(&simple_motor, &status3);
        if (result == RT_EOK)
        {
            LOG_D("Motor Status3:\n");
            LOG_D("  Temperature: %d°C\n", status3.temperature);
            LOG_D("  Current A: %d\n", status3.current_a);
            LOG_D("  Current B: %d\n", status3.current_b);
            LOG_D("  Current C: %d\n", status3.current_c);
        }
        else
        {
            LOG_D("Failed to read status3: %d\n", result);
        }
    }
    else if (rt_strcmp(argv[1], "stats") == 0)
    {
        rt_uint32_t send_count, error_count;
        ms4010_get_statistics(&simple_motor, &send_count, &error_count);
        LOG_D("Communication Statistics:\n");
        LOG_D("  Total Send: %u\n", send_count);
        LOG_D("  Total Error: %u\n", error_count);
        LOG_D("  Success Rate: %.1f%%\n", 
                   send_count > 0 ? ((float)(send_count - error_count) / send_count * 100) : 0);
    }
    else if (rt_strcmp(argv[1], "motor_on") == 0)
    {
        rt_err_t result = ms4010_motor_on(&simple_motor);
        LOG_D("Motor on: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "motor_off") == 0)
    {
        rt_err_t result = ms4010_motor_off(&simple_motor);
        LOG_D("Motor off: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "motor_stop") == 0)
    {
        rt_err_t result = ms4010_motor_stop(&simple_motor);
        LOG_D("Motor stop: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "brake") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple brake <cmd>\n");
            LOG_D("  cmd: 0=lock, 1=release, 16=read status\n");
            return;
        }
        rt_uint8_t brake_cmd = atoi(argv[2]);
        rt_uint8_t brake_status = 0;
        rt_err_t result = ms4010_brake_control(&simple_motor, brake_cmd, &brake_status);
        if (brake_cmd == MS4010_BRAKE_READ_STATUS)
        {
            LOG_D("Brake status read: %s, status=0x%02X\n", result == RT_EOK ? "SUCCESS" : "FAILED", brake_status);
        }
        else
        {
            LOG_D("Brake control cmd=%d: %s\n", brake_cmd, result == RT_EOK ? "SUCCESS" : "FAILED");
        }
    }
    else if (rt_strcmp(argv[1], "clear_err") == 0)
    {
        rt_err_t result = ms4010_clear_error(&simple_motor);
        LOG_D("Clear error: %s\n", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "set_pos") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple set_pos <angle>\n");
            LOG_D("  angle: angle value in 0.01 degree units\n");
            return;
        }
        rt_int32_t angle = atoi(argv[2]);
        rt_err_t result = ms4010_set_current_position(&simple_motor, angle);
        LOG_D("Set current position to %d (0.01°): %s\n", angle, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else
    {
        LOG_D("Unknown command: %s\n", argv[1]);
    }
}
MSH_CMD_EXPORT(ms4010_simple, MS4010 simple test commands);

#endif /* RT_USING_FINSH */

/* 如果需要自动启动测试，取消下面这行的注释 */
// INIT_APP_EXPORT(ms4010_simple_test_start);
