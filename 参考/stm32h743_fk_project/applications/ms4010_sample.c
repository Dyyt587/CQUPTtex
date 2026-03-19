/**
 * @file ms4010_sample.c
 * @brief MS4010电机驱动使用示例
 * @version 1.0
 * @date 2025-07-25
 * @author Your Name
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 展示如何使用MS4010电机驱动进行各种控制
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_ms4010.h"

#define DBG_TAG "ms4010.sample"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* MS4010设备实例 */
static ms4010_device_t ms4010_device;

/* 电机控制线程 */
static void ms4010_control_thread(void *parameter)
{
    rt_err_t result;
    ms4010_status1_t status1;
    ms4010_status_t status2;
    ms4010_status3_t status3;
    rt_uint32_t send_count, error_count;
    
    /* 初始化MS4010电机 */
    result = ms4010_init(&ms4010_device, "uart2", 1, 1000000);
    if (result != RT_EOK)
    {
        LOG_E("Failed to initialize MS4010 motor");
        return;
    }
    
    LOG_I("MS4010 motor initialized successfully");
    
    /* 电机控制循环 */
    rt_uint32_t tick = 0;
    rt_int16_t speed_target = 0;
    rt_int16_t direction = 1;
    
    while (1)
    {
        switch (tick % 500)
        {
            case 0:
                /* 开环控制测试 */
                LOG_I("Testing open loop control");
                ms4010_open_loop_control(&ms4010_device, 300 * direction);
                break;
                
            case 100:
                /* 速度控制测试 */
                LOG_I("Testing speed control");
                speed_target = 360 * direction; // 360 dps
                ms4010_speed_control(&ms4010_device, speed_target);
                break;
                
            case 200:
                /* 位置控制测试 */
                LOG_I("Testing position control");
                ms4010_position_control2(&ms4010_device, 18000 * direction, 36000); // 180度, 360dps
                break;
                
            case 300:
                /* 读取状态1和状态2 */
                LOG_I("Reading status1 and status2");
                result = ms4010_read_status1(&ms4010_device, &status1);
                rt_err_t result2 = ms4010_get_status(&ms4010_device, &status2);
                if (result == RT_EOK && result2 == RT_EOK)
                {
                    LOG_I("Status1 - Temp: %d°C, Voltage: %.2fV, Current: %.2fA", 
                          status1.temperature, status1.voltage * 0.01f, status1.current * 0.01f);
                    LOG_I("Status2 - Speed: %d dps, Encoder: %u", status2.speed, status2.encoder);
                }
                break;
                
            case 400:
                /* 停止电机 */
                LOG_I("Stopping motor");
                ms4010_motor_stop(&ms4010_device);
                direction = -direction; // 改变方向
                break;
        }
        
        /* 每20次循环读取一次状态3 */
        if (tick % 20 == 0)
        {
            result = ms4010_read_status3(&ms4010_device, &status3);
            if (result == RT_EOK)
            {
                LOG_D("Status3 - Temp: %d°C, Current_A: %d, Current_B: %d, Current_C: %d", 
                      status3.temperature, status3.current_a, status3.current_b, status3.current_c);
            }
        }
        
        /* 每50次循环显示统计信息 */
        if (tick % 50 == 0)
        {
            ms4010_get_statistics(&ms4010_device, &send_count, &error_count);
            LOG_I("Statistics - Send: %d, Error: %d, Success Rate: %.1f%%", 
                  send_count, error_count,
                  send_count > 0 ? ((float)(send_count - error_count) / send_count * 100) : 0);
        }
        
        tick++;
        rt_thread_mdelay(100); // 100ms周期
    }
}

/* 初始化MS4010示例 */
static int ms4010_sample_init(void)
{
    rt_thread_t tid;
    
    tid = rt_thread_create("ms4010_ctrl", ms4010_control_thread, RT_NULL, 2048, 10, 20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        LOG_I("MS4010 control thread started");
    }
    else
    {
        LOG_E("Failed to create MS4010 control thread");
        return -RT_ERROR;
    }
    
    return RT_EOK;
}
//INIT_APP_EXPORT(ms4010_sample_init);

/* MSH命令支持 */
#ifdef RT_USING_FINSH

static void ms4010_sample(int argc, char **argv)
{
    rt_err_t result;
    ms4010_status1_t status1;
    ms4010_status_t status2;
    ms4010_status3_t status3;
    rt_uint32_t send_count, error_count;
    
    if (argc < 2)
    {
        rt_kprintf("Usage: ms4010_sample <command> [value]\n");
        rt_kprintf("Commands:\n");
        rt_kprintf("  init           - Initialize MS4010 motor\n");
        rt_kprintf("  deinit         - Deinitialize MS4010 motor\n");
        rt_kprintf("  status1        - Get motor status1 (temperature, voltage, current, state)\n");
        rt_kprintf("  status2        - Get motor status2 (temperature, speed, encoder, power)\n");
        rt_kprintf("  status3        - Get motor status3 (temperature, three-phase currents)\n");
        rt_kprintf("  open <value>   - Open loop control (-1000~1000)\n");
        rt_kprintf("  speed <value>  - Speed control (dps)\n");
        rt_kprintf("  torque <value> - Torque control\n");
        rt_kprintf("  pos <angle> [speed] - Position control (0.01°, opt speed)\n");
        rt_kprintf("  pos2 <angle> <speed> - Position control2 (0.01°, with max speed)\n");
        rt_kprintf("  single1 <dir> <angle> - Single turn pos control1\n");
        rt_kprintf("  single2 <dir> <angle> [speed] - Single turn pos control2\n");
        rt_kprintf("  inc1 <angle>   - Increment position control1\n");
        rt_kprintf("  inc2 <angle> [speed] - Increment position control2\n");
        rt_kprintf("  motor_on       - Turn motor on\n");
        rt_kprintf("  motor_off      - Turn motor off\n");
        rt_kprintf("  motor_stop     - Stop motor immediately\n");
        rt_kprintf("  brake <cmd>    - Brake control (0=lock, 1=release, 16=read)\n");
        rt_kprintf("  clear_error    - Clear error flags\n");
        rt_kprintf("  set_pos <angle> - Set current position to specified angle (0.01°)\n");
        rt_kprintf("  stats          - Show communication statistics\n");
        return;
    }
    
    if (rt_strcmp(argv[1], "init") == 0)
    {
        result = ms4010_init(&ms4010_device, "uart2", 1, 115200);
        rt_kprintf("MS4010 init %s\n", result == RT_EOK ? "success" : "failed");
    }
    else if (rt_strcmp(argv[1], "deinit") == 0)
    {
        result = ms4010_deinit(&ms4010_device);
        rt_kprintf("MS4010 deinit %s\n", result == RT_EOK ? "success" : "failed");
    }
    
    if (rt_strcmp(argv[1], "status1") == 0)
    {
        result = ms4010_read_status1(&ms4010_device, &status1);
        if (result == RT_EOK)
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
            rt_kprintf("Failed to get motor status1\n");
        }
    }
    else if (rt_strcmp(argv[1], "status2") == 0)
    {
        result = ms4010_get_status(&ms4010_device, &status2);
        if (result == RT_EOK)
        {
            rt_kprintf("Motor Status2:\n");
            rt_kprintf("  Temperature: %d°C\n", status2.temperature);
            rt_kprintf("  Power/Torque: %d\n", status2.power_or_torque);
            rt_kprintf("  Speed: %d dps\n", status2.speed);
            rt_kprintf("  Encoder: %u\n", status2.encoder);
        }
        else
        {
            rt_kprintf("Failed to get motor status2\n");
        }
    }
    else if (rt_strcmp(argv[1], "status3") == 0)
    {
        result = ms4010_read_status3(&ms4010_device, &status3);
        if (result == RT_EOK)
        {
            rt_kprintf("Motor Status3:\n");
            rt_kprintf("  Temperature: %d°C\n", status3.temperature);
            rt_kprintf("  Current A: %d\n", status3.current_a);
            rt_kprintf("  Current B: %d\n", status3.current_b);
            rt_kprintf("  Current C: %d\n", status3.current_c);
        }
        else
        {
            rt_kprintf("Failed to get motor status3\n");
        }
    }
    else if (rt_strcmp(argv[1], "open") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify open loop value (-1000~1000)\n");
            return;
        }
        
        rt_int16_t value = atoi(argv[2]);
        result = ms4010_open_loop_control(&ms4010_device, value);
        rt_kprintf("Open loop control %s (value: %d)\n", 
                   result == RT_EOK ? "success" : "failed", value);
    }
    else if (rt_strcmp(argv[1], "speed") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify speed value (dps)\n");
            return;
        }
        
        rt_int32_t speed = atoi(argv[2]);
        result = ms4010_speed_control(&ms4010_device, speed);
        rt_kprintf("Speed control %s (speed: %d dps)\n", 
                   result == RT_EOK ? "success" : "failed", speed);
    }
    else if (rt_strcmp(argv[1], "torque") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify torque value\n");
            return;
        }
        
        rt_int16_t torque = atoi(argv[2]);
        result = ms4010_torque_control(&ms4010_device, torque);
        rt_kprintf("Torque control %s (torque: %d)\n", 
                   result == RT_EOK ? "success" : "failed", torque);
    }
    else if (rt_strcmp(argv[1], "pos") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify position value (0.01 degree)\n");
            return;
        }
        
        rt_int32_t angle = atoi(argv[2]);
        rt_uint16_t speed = (argc >= 4) ? atoi(argv[3]) : 18000;
        result = ms4010_position_control(&ms4010_device, angle, speed);
        rt_kprintf("Position control %s (angle: %d * 0.01°, speed: %d)\n", 
                   result == RT_EOK ? "success" : "failed", angle, speed);
    }
    else if (rt_strcmp(argv[1], "pos2") == 0)
    {
        if (argc < 4)
        {
            rt_kprintf("Please specify angle and max speed\n");
            return;
        }
        
        rt_int64_t angle = atoll(argv[2]);
        rt_uint32_t max_speed = atoi(argv[3]);
        result = ms4010_position_control2(&ms4010_device, angle, max_speed);
        rt_kprintf("Position control2 %s (angle: %lld * 0.01°, max_speed: %u * 0.01dps)\n", 
                   result == RT_EOK ? "success" : "failed", angle, max_speed);
    }
    else if (rt_strcmp(argv[1], "single1") == 0)
    {
        if (argc < 4)
        {
            rt_kprintf("Please specify direction and angle\n");
            return;
        }
        
        rt_uint8_t direction = atoi(argv[2]);
        rt_uint16_t angle = atoi(argv[3]);
        result = ms4010_single_position_control1(&ms4010_device, direction, angle);
        rt_kprintf("Single position control1 %s (dir: %d, angle: %d * 0.01°)\n", 
                   result == RT_EOK ? "success" : "failed", direction, angle);
    }
    else if (rt_strcmp(argv[1], "single2") == 0)
    {
        if (argc < 4)
        {
            rt_kprintf("Please specify direction and angle\n");
            return;
        }
        
        rt_uint8_t direction = atoi(argv[2]);
        rt_uint16_t angle = atoi(argv[3]);
        rt_uint32_t max_speed = (argc >= 5) ? atoi(argv[4]) : 18000;
        result = ms4010_single_position_control2(&ms4010_device, direction, angle, max_speed);
        rt_kprintf("Single position control2 %s (dir: %d, angle: %d * 0.01°, max_speed: %u)\n", 
                   result == RT_EOK ? "success" : "failed", direction, angle, max_speed);
    }
    else if (rt_strcmp(argv[1], "inc1") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify angle increment\n");
            return;
        }
        
        rt_int32_t angle_increment = atoi(argv[2]);
        result = ms4010_increment_position_control1(&ms4010_device, angle_increment);
        rt_kprintf("Increment position control1 %s (increment: %d * 0.01°)\n", 
                   result == RT_EOK ? "success" : "failed", angle_increment);
    }
    else if (rt_strcmp(argv[1], "inc2") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify angle increment\n");
            return;
        }
        
        rt_int32_t angle_increment = atoi(argv[2]);
        rt_uint32_t max_speed = (argc >= 4) ? atoi(argv[3]) : 36000;
        result = ms4010_increment_position_control2(&ms4010_device, angle_increment, max_speed);
        rt_kprintf("Increment position control2 %s (increment: %d * 0.01°, max_speed: %u)\n", 
                   result == RT_EOK ? "success" : "failed", angle_increment, max_speed);
    }
    }
    else if (rt_strcmp(argv[1], "motor_on") == 0)
    {
        result = ms4010_motor_on(&ms4010_device);
        rt_kprintf("Motor on %s\n", result == RT_EOK ? "success" : "failed");
    }
    else if (rt_strcmp(argv[1], "motor_off") == 0)
    {
        result = ms4010_motor_off(&ms4010_device);
        rt_kprintf("Motor off %s\n", result == RT_EOK ? "success" : "failed");
    }
    else if (rt_strcmp(argv[1], "motor_stop") == 0)
    {
        result = ms4010_motor_stop(&ms4010_device);
        rt_kprintf("Motor stop %s\n", result == RT_EOK ? "success" : "failed");
    }
    else if (rt_strcmp(argv[1], "brake") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify brake command (0=lock, 1=release, 16=read)\n");
            return;
        }
        
        rt_uint8_t brake_cmd = atoi(argv[2]);
        rt_uint8_t brake_status = 0;
        result = ms4010_brake_control(&ms4010_device, brake_cmd, &brake_status);
        if (brake_cmd == MS4010_BRAKE_READ_STATUS)
        {
            rt_kprintf("Brake status read %s (status: 0x%02X)\n", 
                       result == RT_EOK ? "success" : "failed", brake_status);
        }
        else
        {
            rt_kprintf("Brake control %s (command: %d)\n", 
                       result == RT_EOK ? "success" : "failed", brake_cmd);
        }
    }
    else if (rt_strcmp(argv[1], "clear_error") == 0)
    {
        result = ms4010_clear_error(&ms4010_device);
        rt_kprintf("Clear error %s\n", result == RT_EOK ? "success" : "failed");
    }
    else if (rt_strcmp(argv[1], "set_pos") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Please specify angle value (0.01 degree units)\n");
            return;
        }
        
        rt_int32_t angle = atoi(argv[2]);
        result = ms4010_set_current_position(&ms4010_device, angle);
        rt_kprintf("Set current position to %d (0.01°): %s\n", angle, result == RT_EOK ? "success" : "failed");
    }
    else if (rt_strcmp(argv[1], "stats") == 0)
    {
        ms4010_get_statistics(&ms4010_device, &send_count, &error_count);
        rt_kprintf("Communication Statistics:\n");
        rt_kprintf("  Total Send: %u\n", send_count);
        rt_kprintf("  Total Error: %u\n", error_count);
        rt_kprintf("  Success Rate: %.1f%%\n", 
                   send_count > 0 ? ((float)(send_count - error_count) / send_count * 100) : 0);
    }
    else
    {
        rt_kprintf("Unknown command: %s\n", argv[1]);
    }
}
MSH_CMD_EXPORT(ms4010_sample, MS4010 motor sample commands);

#endif /* RT_USING_FINSH */
