/**
 * @file ms4010_multi_test.c
 * @brief MS4010多电机测试文件
 * @version 1.0
 * @date 2025-07-28
 * @author Dyyt587
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 基于MS4010驱动的多电机测试程序，支持在一个RS485总线上控制多个电机
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include "drv_ms4010.h"

#define DBG_TAG "ms4010_multi"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* 配置参数 */
#define MS4010_RS485_NAME               "uart2"     // RS485设备名称
#define MS4010_BAUDRATE                 1000000      // 波特率
#define MS4010_TEST_THREAD_STACK        2048        // 线程栈大小
#define MS4010_TEST_THREAD_PRIORITY     15          // 线程优先级
#define MAX_TEST_MOTORS                 8           // 测试支持的最大电机数

/* 全局变量 */
static ms4010_manager_t g_ms4010_manager;           // MS4010管理器实例
static ms4010_device_t g_motors[MAX_TEST_MOTORS];   // 电机设备数组
static rt_thread_t g_test_thread = RT_NULL;         // 测试线程句柄
static rt_bool_t g_test_running = RT_FALSE;         // 测试运行标志
static rt_uint8_t g_active_motors = 0;              // 当前活动电机数量

/**
 * @brief 初始化多电机系统
 */
static rt_err_t ms4010_multi_init(void)
{
    rt_err_t result;

    LOG_I("=== MS4010 Multi-Motor System Initialize ===");

    /* 初始化管理器 */
    result = ms4010_manager_init(&g_ms4010_manager, MS4010_RS485_NAME, MS4010_BAUDRATE);
    if (result != RT_EOK)
    {
        LOG_E("Failed to initialize MS4010 manager: %d", result);
        return result;
    }

    LOG_I("MS4010 manager initialized successfully");
    return RT_EOK;
}

/**
 * @brief 反初始化多电机系统
 */
static rt_err_t ms4010_multi_deinit(void)
{
    LOG_I("=== MS4010 Multi-Motor System Deinitialize ===");

    /* 移除所有电机 */
    for (int i = 0; i < g_active_motors; i++)
    {
        ms4010_manager_remove_motor(&g_ms4010_manager, g_motors[i].motor_id);
    }
    g_active_motors = 0;

    /* 反初始化管理器 */
    rt_err_t result = ms4010_manager_deinit(&g_ms4010_manager);
    if (result != RT_EOK)
    {
        LOG_E("Failed to deinitialize MS4010 manager: %d", result);
        return result;
    }

    LOG_I("MS4010 manager deinitialized successfully");
    return RT_EOK;
}

/**
 * @brief 同步活动电机数组与管理器
 */
static void sync_active_motors_from_manager(void)
{
    g_active_motors = 0;
    
    /* 遍历所有可能的电机ID */
    for (rt_uint8_t id = 1; id <= 254 && g_active_motors < MAX_TEST_MOTORS; id++)
    {
        ms4010_device_t *motor = ms4010_manager_get_motor(&g_ms4010_manager, id);
        if (motor != RT_NULL)
        {
            g_motors[g_active_motors] = *motor;
            g_active_motors++;
        }
    }
    
    LOG_I("Synced active motors array: %d motors", g_active_motors);
}

/**
 * @brief 添加电机到系统
 */
static rt_err_t ms4010_add_motor(rt_uint8_t motor_id)
{
    if (g_active_motors >= MAX_TEST_MOTORS)
    {
        LOG_E("Maximum motor count reached");
        return -RT_ERROR;
    }

    /* 动态分配电机设备内存以保持一致性 */
    ms4010_device_t *device = (ms4010_device_t*)rt_malloc(sizeof(ms4010_device_t));
    if (device == RT_NULL)
    {
        LOG_E("Failed to allocate memory for motor %d", motor_id);
        return -RT_ENOMEM;
    }

    rt_err_t result = ms4010_manager_add_motor(&g_ms4010_manager, device, motor_id);
    if (result != RT_EOK)
    {
        LOG_E("Failed to add motor %d: %d", motor_id, result);
        rt_free(device);
        return result;
    }

    /* 同步活动电机数组 */
    sync_active_motors_from_manager();
    
    LOG_I("Motor %d added successfully (Total: %d motors)", motor_id, g_active_motors);
    return RT_EOK;
}

/**
 * @brief 基础多电机通信测试
 */
static rt_err_t ms4010_multi_basic_test(void)
{
    LOG_I("=== MS4010 Multi-Motor Basic Test ===");

    /* 测试所有电机的状态读取 */
    rt_err_t result = ms4010_manager_get_all_status(&g_ms4010_manager);
    if (result != RT_EOK)
    {
        LOG_W("Some motors failed status check");
    }

    /* 逐个测试电机控制 */
    for (int i = 0; i < g_active_motors; i++)
    {
        ms4010_device_t *motor = &g_motors[i];
        
        LOG_I("Testing motor %d control...", motor->motor_id);
        
        /* 开启电机 */
        result = ms4010_motor_on(motor);
        if (result != RT_EOK)
        {
            LOG_E("Failed to turn on motor %d: %d", motor->motor_id, result);
            continue;
        }
        
        rt_thread_mdelay(100);
        
        /* 简单位置控制测试 */
        result = ms4010_position_control(motor, 0, 1000);
        if (result != RT_EOK)
        {
            LOG_E("Failed position control for motor %d: %d", motor->motor_id, result);
        }
        
        rt_thread_mdelay(500);
        
        /* 关闭电机 */
        ms4010_motor_off(motor);
        rt_thread_mdelay(100);
    }

    return RT_EOK;
}

/**
 * @brief 同步控制测试
 */
static rt_err_t ms4010_sync_control_test(void)
{
    LOG_I("=== MS4010 Synchronized Control Test ===");

    if (g_active_motors < 2)
    {
        LOG_W("Need at least 2 motors for sync test");
        return -RT_ERROR;
    }

    /* 同时开启所有电机 */
    for (int i = 0; i < g_active_motors; i++)
    {
        ms4010_motor_on(&g_motors[i]);
        rt_thread_mdelay(50);
    }

    /* 同步位置控制 */
    rt_int32_t positions[] = {0, 900, 1800, 2700, 0, -900, -1800, -2700};
    
    for (int pos = 0; pos < 4; pos++)
    {
        LOG_I("Moving all motors to position %d...", pos);
        
        /* 同时发送位置命令 */
        for (int i = 0; i < g_active_motors; i++)
        {
            ms4010_position_control(&g_motors[i], positions[pos % 8], 2000);
            rt_thread_mdelay(10); // 短暂延时避免总线冲突
        }
        
        rt_thread_mdelay(2000); // 等待运动完成
    }

    /* 同时关闭所有电机 */
    for (int i = 0; i < g_active_motors; i++)
    {
        ms4010_motor_off(&g_motors[i]);
        rt_thread_mdelay(50);
    }

    return RT_EOK;
}

/**
 * @brief 性能测试
 */
static rt_err_t ms4010_performance_test(void)
{
    LOG_I("=== MS4010 Performance Test ===");

    rt_uint32_t start_time = rt_tick_get();
    rt_uint32_t total_commands = 0;
    rt_uint32_t failed_commands = 0;

    /* 连续发送状态查询命令 */
    for (int round = 0; round < 10; round++)
    {
        for (int i = 0; i < g_active_motors; i++)
        {
            rt_err_t result = ms4010_read_status1(&g_motors[i], &g_motors[i].status1);
            total_commands++;
            if (result != RT_EOK)
            {
                failed_commands++;
            }
            rt_thread_mdelay(10);
        }
    }

    rt_uint32_t end_time = rt_tick_get();
    rt_uint32_t elapsed_ms = (end_time - start_time) * 1000 / RT_TICK_PER_SECOND;

    LOG_I("Performance Test Results:");
    LOG_I("  Total commands: %d", total_commands);
    LOG_I("  Failed commands: %d", failed_commands);
    LOG_I("  Success rate: %d.%d%%", 
          (total_commands - failed_commands) * 100 / total_commands,
          ((total_commands - failed_commands) * 1000 / total_commands) % 10);
    LOG_I("  Total time: %d ms", elapsed_ms);
    LOG_I("  Average time per command: %d ms", elapsed_ms / total_commands);

    return RT_EOK;
}

/**
 * @brief 多电机测试线程
 */
static void ms4010_multi_test_thread(void *parameter)
{
    LOG_I("MS4010 multi-motor test thread started");

    while (g_test_running)
    {
        LOG_I("=== Starting Multi-Motor Test Cycle ===");

        /* 基础测试 */
        ms4010_multi_basic_test();
        rt_thread_mdelay(2000);

        if (!g_test_running) break;

        /* 同步控制测试 */
        ms4010_sync_control_test();
        rt_thread_mdelay(3000);

        if (!g_test_running) break;

        /* 性能测试 */
        ms4010_performance_test();
        rt_thread_mdelay(5000);

        if (!g_test_running) break;
    }

    LOG_I("MS4010 multi-motor test thread stopped");
}

/**
 * @brief MS4010多电机测试命令处理函数
 */
static void ms4010_multi_test_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("MS4010 Multi-Motor Test Commands:\n");
        LOG_D("  ms4010_multi init                    - Initialize multi-motor system\n");
        LOG_D("  ms4010_multi deinit                  - Deinitialize multi-motor system\n");
        LOG_D("  ms4010_multi add <id>                - Add motor with specified ID\n");
        LOG_D("  ms4010_multi remove <id>             - Remove motor by ID\n");
        LOG_D("  ms4010_multi scan [start] [end] [timeout] - Auto scan motors (default: 1-254, 100ms)\n");
        LOG_D("  ms4010_multi qscan              - Quick scan common ID ranges\n");
        LOG_D("  ms4010_multi clear_all               - Remove all motors from manager\n");
        LOG_D("  ms4010_multi list                    - List all motors\n");
        LOG_D("  ms4010_multi status                  - Get all motors status\n");
        LOG_D("  ms4010_multi test_basic              - Run basic communication test\n");
        LOG_D("  ms4010_multi test_sync               - Run synchronized control test\n");
        LOG_D("  ms4010_multi test_performance        - Run performance test\n");
        LOG_D("  ms4010_multi start_auto              - Start automatic test thread\n");
        LOG_D("  ms4010_multi stop_auto               - Stop automatic test thread\n");
        LOG_D("  ms4010_multi motor <id> <cmd> [args] - Control specific motor\n");
        LOG_D("\nMotor Control Commands:\n");
        LOG_D("  motor <id> on                        - Turn on motor\n");
        LOG_D("  motor <id> off                       - Turn off motor\n");
        LOG_D("  motor <id> pos <angle>               - Position control\n");
        LOG_D("  motor <id> speed <dps>               - Speed control\n");
        LOG_D("  motor <id> status                    - Get motor status\n");
        return;
    }

    if (rt_strcmp(argv[1], "init") == 0)
    {
        rt_err_t result = ms4010_multi_init();
        LOG_D("Multi-motor system init: %s\n", (result == RT_EOK) ? "OK" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "deinit") == 0)
    {
        rt_err_t result = ms4010_multi_deinit();
        LOG_D("Multi-motor system deinit: %s\n", (result == RT_EOK) ? "OK" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "add") == 0)
    {
        if (argc != 3)
        {
            LOG_D("Usage: %s add <motor_id>\n", argv[0]);
            return;
        }
        
        rt_uint8_t motor_id = atoi(argv[2]);
        rt_err_t result = ms4010_add_motor(motor_id);
        LOG_D("Add motor %d: %s\n", motor_id, (result == RT_EOK) ? "OK" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "remove") == 0)
    {
        if (argc != 3)
        {
            LOG_D("Usage: %s remove <motor_id>\n", argv[0]);
            return;
        }
        
        rt_uint8_t motor_id = atoi(argv[2]);
        rt_err_t result = ms4010_manager_remove_motor(&g_ms4010_manager, motor_id);
        LOG_D("Remove motor %d: %s\n", motor_id, (result == RT_EOK) ? "OK" : "FAILED");
        
        /* 同步活动电机数组 */
        if (result == RT_EOK)
        {
            sync_active_motors_from_manager();
        }
    }
    else if (rt_strcmp(argv[1], "scan") == 0)
    {
        rt_uint8_t start_id = 1;
        rt_uint8_t end_id = 254;
        rt_uint32_t timeout_ms = 100;
        
        /* 解析可选参数 */
        if (argc >= 3) start_id = atoi(argv[2]);
        if (argc >= 4) end_id = atoi(argv[3]);
        if (argc >= 5) timeout_ms = atoi(argv[4]);
        
        /* 参数范围检查 */
        if (start_id < 1 || end_id > 254 || start_id > end_id)
        {
            LOG_D("Invalid scan range. Use: scan [start_id] [end_id] [timeout_ms]\n");
            LOG_D("  start_id: 1-254, end_id: 1-254, timeout_ms: 10-1000\n");
            return;
        }
        
        if (timeout_ms < 10 || timeout_ms > 1000)
        {
            LOG_D("Invalid timeout. Range: 10-1000ms\n");
            return;
        }
        
        LOG_D("Scanning motors from ID %d to %d (timeout: %dms)...\n", start_id, end_id, timeout_ms);
        rt_uint8_t found_count = ms4010_manager_auto_scan(&g_ms4010_manager, start_id, end_id, timeout_ms);
        LOG_D("Auto scan complete: %d motors found\n", found_count);
        
        /* 同步活动电机数组 */
        sync_active_motors_from_manager();
    }
    else if (rt_strcmp(argv[1], "qscan") == 0)
    {
        LOG_D("Starting quick scan for motors...\n");
        rt_uint8_t found_count = ms4010_manager_quick_scan(&g_ms4010_manager);
        LOG_D("Quick scan complete: %d motors found\n", found_count);
        
        /* 同步活动电机数组 */
        sync_active_motors_from_manager();
    }
    else if (rt_strcmp(argv[1], "clear_all") == 0)
    {
        rt_err_t result = ms4010_manager_clear_all(&g_ms4010_manager);
        LOG_D("Clear all motors: %s\n", (result == RT_EOK) ? "OK" : "FAILED");
        
        /* 清空活动电机数组 */
        g_active_motors = 0;
        rt_memset(g_motors, 0, sizeof(g_motors));
    }
    else if (rt_strcmp(argv[1], "list") == 0)
    {
        LOG_D("Active motors: %d\n", g_active_motors);
        for (int i = 0; i < g_active_motors; i++)
        {
            LOG_D("  Motor %d: ID=%d, Send=%d, Error=%d\n", 
                      i, g_motors[i].motor_id, g_motors[i].send_count, g_motors[i].error_count);
        }
    }
    else if (rt_strcmp(argv[1], "status") == 0)
    {
        ms4010_manager_get_all_status(&g_ms4010_manager);
    }
    else if (rt_strcmp(argv[1], "test_basic") == 0)
    {
        ms4010_multi_basic_test();
    }
    else if (rt_strcmp(argv[1], "test_sync") == 0)
    {
        ms4010_sync_control_test();
    }
    else if (rt_strcmp(argv[1], "test_performance") == 0)
    {
        ms4010_performance_test();
    }
    else if (rt_strcmp(argv[1], "start_auto") == 0)
    {
        if (g_test_thread != RT_NULL)
        {
            LOG_D("Test thread is already running\n");
            return;
        }
        
        g_test_running = RT_TRUE;
        g_test_thread = rt_thread_create("ms4010_multi_test", 
                                        ms4010_multi_test_thread, RT_NULL,
                                        MS4010_TEST_THREAD_STACK, MS4010_TEST_THREAD_PRIORITY, 20);
        if (g_test_thread != RT_NULL)
        {
            rt_thread_startup(g_test_thread);
            LOG_D("Multi-motor test thread started\n");
        }
        else
        {
            g_test_running = RT_FALSE;
            LOG_D("Failed to create test thread\n");
        }
    }
    else if (rt_strcmp(argv[1], "stop_auto") == 0)
    {
        if (g_test_thread == RT_NULL)
        {
            LOG_D("Test thread is not running\n");
            return;
        }
        
        g_test_running = RT_FALSE;
        rt_thread_delete(g_test_thread);
        g_test_thread = RT_NULL;
        LOG_D("Multi-motor test thread stopped\n");
    }
    else if (rt_strcmp(argv[1], "motor") == 0)
    {
        if (argc < 4)
        {
            LOG_D("Usage: %s motor <motor_id> <command> [args]\n", argv[0]);
            return;
        }
        
        rt_uint8_t motor_id = atoi(argv[2]);
        ms4010_device_t *motor = ms4010_manager_get_motor(&g_ms4010_manager, motor_id);
        
        if (motor == RT_NULL)
        {
            LOG_D("Motor %d not found\n", motor_id);
            return;
        }
        
        if (rt_strcmp(argv[3], "on") == 0)
        {
            rt_err_t result = ms4010_motor_on(motor);
            LOG_D("Motor %d turn on: %s\n", motor_id, (result == RT_EOK) ? "OK" : "FAILED");
        }
        else if (rt_strcmp(argv[3], "off") == 0)
        {
            rt_err_t result = ms4010_motor_off(motor);
            LOG_D("Motor %d turn off: %s\n", motor_id, (result == RT_EOK) ? "OK" : "FAILED");
        }
        else if (rt_strcmp(argv[3], "pos") == 0)
        {
            if (argc != 5)
            {
                LOG_D("Usage: %s motor %d pos <angle>\n", argv[0], motor_id);
                return;
            }
            
            rt_int32_t angle = atoi(argv[4]);
            rt_err_t result = ms4010_position_control(motor, angle, 2000);
            LOG_D("Motor %d position control to %d: %s\n", 
                      motor_id, angle, (result == RT_EOK) ? "OK" : "FAILED");
        }
        else if (rt_strcmp(argv[3], "speed") == 0)
        {
            if (argc != 5)
            {
                LOG_D("Usage: %s motor %d speed <dps>\n", argv[0], motor_id);
                return;
            }
            
            rt_int32_t speed = atoi(argv[4]);
            rt_err_t result = ms4010_speed_control(motor, speed);
            LOG_D("Motor %d speed control to %d dps: %s\n", 
                      motor_id, speed, (result == RT_EOK) ? "OK" : "FAILED");
        }
        else if (rt_strcmp(argv[3], "status") == 0)
        {
            rt_err_t result = ms4010_read_status1(motor, &motor->status1);
            if (result == RT_EOK)
            {
                LOG_D("Motor %d status:\n", motor_id);
                LOG_D("  Temperature: %d°C\n", motor->status1.temperature);
                LOG_D("  Voltage: %d.%02dV\n", motor->status1.voltage / 100, motor->status1.voltage % 100);
                LOG_D("  Current: %d.%02dA\n", motor->status1.current / 100, motor->status1.current % 100);
                LOG_D("  Motor State: 0x%02X\n", motor->status1.motor_state);
                LOG_D("  Error State: 0x%02X\n", motor->status1.error_state);
            }
            else
            {
                LOG_D("Motor %d status read failed: %d\n", motor_id, result);
            }
        }
        else
        {
            LOG_D("Unknown motor command: %s\n", argv[3]);
        }
    }
    else
    {
        LOG_D("Unknown command: %s\n", argv[1]);
    }
}

/* 注册MSH命令 */
MSH_CMD_EXPORT_ALIAS(ms4010_multi_test_cmd, ms4010_multi, MS4010 multi-motor test commands);
