/**
 * @file drv_ms4010.c
 * @brief MS4010电机RS485驱动实现
 * @version 1.0
 * @date 2025-07-25
 * @author Your Name
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 基于RT-Thread RS485库实现MS4010电机驱动
 */

#include "drv_ms4010.h"

#define DBG_TAG "ms4010"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define USART_NAME "uart2"

/* 静态函数前向声明 */
static rt_err_t ms4010_send_command(ms4010_device_t *device, rt_uint8_t cmd, rt_uint8_t data_size, const rt_uint8_t *data, rt_uint8_t expected_response_size);
static rt_err_t ms4010_send_command_shared(rs485_inst_t *rs485_handle, rt_mutex_t bus_mutex, rt_uint8_t motor_id, rt_uint8_t cmd, rt_uint8_t data_size, const rt_uint8_t *data, rt_uint8_t expected_response_size, rt_uint8_t *rx_buffer);
static rt_err_t ms4010_pack_frame(rt_uint8_t *buffer, rt_uint8_t cmd, rt_uint8_t id, rt_uint8_t data_size, const rt_uint8_t *data, rt_uint8_t *frame_size);
static rt_err_t ms4010_parse_response(ms4010_device_t *device, const rt_uint8_t *buffer, rt_uint8_t size);
static rt_err_t ms4010_parse_status1_response(ms4010_device_t *device, const rt_uint8_t *buffer, rt_uint8_t size);
static rt_err_t ms4010_parse_status3_response(ms4010_device_t *device, const rt_uint8_t *buffer, rt_uint8_t size);
static rt_uint8_t ms4010_calculate_checksum(const rt_uint8_t *buffer, rt_uint8_t size);
static rt_err_t ms4010_clear_rx_buffer(rs485_inst_t *rs485_handle);

/* 多电机管理器实现 */

/**
 * @brief 初始化MS4010多电机管理器
 */
rt_err_t ms4010_manager_init(ms4010_manager_t *manager, const char *rs485_name, rt_uint32_t baudrate)
{
    RT_ASSERT(manager != RT_NULL);
    RT_ASSERT(rs485_name != RT_NULL);

    /* 清零管理器结构体 */
    rt_memset(manager, 0, sizeof(ms4010_manager_t));

    /* 创建RS485实例 */
    manager->rs485_handle = rs485_create(rs485_name, baudrate, RS485_SAMPLE_MASTER_PARITY, RS485_SAMPLE_MASTER_PIN, RS485_SAMPLE_MASTER_LVL);
    if (manager->rs485_handle == RT_NULL)
    {
        LOG_E("Failed to create RS485 instance for %s", rs485_name);
        return -MS4010_ERROR;
    }
		//修改波特率
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
		
    config.baud_rate = baudrate;
    config.data_bits = DATA_BITS_8;
    config.parity = PARITY_NONE;
    config.stop_bits =STOP_BITS_1;
    config.tx_bufsz= BSP_UART2_TX_BUFSIZE;
		config.rx_bufsz=BSP_UART2_RX_BUFSIZE;
		LOG_E("USing uart2 TX RX buffer");
		//rt_device_control(hinst->serial, RT_DEVICE_CTRL_CONFIG, &config);
		if(rt_device_control(rt_device_find(USART_NAME), RT_DEVICE_CTRL_CONFIG, &config)!=RT_EOK)
		{
			LOG_E("%s Buad Change Error",USART_NAME);
		}
		else{
			LOG_E("%s Buad Change OK",USART_NAME);
		}
		//修改波特率
		
    /* 设置超时时间 */
    rs485_set_recv_tmo(manager->rs485_handle, MS4010_DEFAULT_TIMEOUT);

    /* 连接RS485 */
    if (rs485_connect(manager->rs485_handle) != RT_EOK)
    {
        LOG_E("Failed to connect RS485");
        rs485_destory(manager->rs485_handle);
        return -MS4010_ERROR;
    }

    /* 创建总线互斥锁 */
    manager->bus_mutex = rt_mutex_create("ms4010_bus_mutex", RT_IPC_FLAG_FIFO);
    if (manager->bus_mutex == RT_NULL)
    {
        LOG_E("Failed to create bus mutex");
        rs485_disconn(manager->rs485_handle);
        rs485_destory(manager->rs485_handle);
        return -MS4010_ERROR;
    }

    /* 创建管理器互斥锁 */
    manager->manager_mutex = rt_mutex_create("ms4010_mgr_mutex", RT_IPC_FLAG_FIFO);
    if (manager->manager_mutex == RT_NULL)
    {
        LOG_E("Failed to create manager mutex");
        rt_mutex_delete(manager->bus_mutex);
        rs485_disconn(manager->rs485_handle);
        rs485_destory(manager->rs485_handle);
        return -MS4010_ERROR;
    }

    manager->motor_count = 0;
    manager->initialized = RT_TRUE;

    LOG_I("MS4010 manager initialized successfully on %s", rs485_name);
    return RT_EOK;
}

/**
 * @brief 反初始化MS4010多电机管理器
 */
rt_err_t ms4010_manager_deinit(ms4010_manager_t *manager)
{
    RT_ASSERT(manager != RT_NULL);

    if (!manager->initialized)
    {
        return RT_EOK;
    }

    /* 移除所有电机并释放内存 */
    for (int i = 0; i < MS4010_MAX_MOTORS; i++)
    {
        if (manager->motors[i] != RT_NULL)
        {
            ms4010_device_t *device = manager->motors[i];
            
            /* 释放设备互斥锁 */
            if (device->mutex != RT_NULL)
            {
                rt_mutex_delete(device->mutex);
                device->mutex = RT_NULL;
            }
            
            device->rs485_handle = RT_NULL; // 防止单独释放
            
            /* 释放动态分配的设备内存 */
            rt_free(device);
            manager->motors[i] = RT_NULL;
        }
    }

    /* 释放总线互斥锁 */
    if (manager->bus_mutex != RT_NULL)
    {
        rt_mutex_delete(manager->bus_mutex);
        manager->bus_mutex = RT_NULL;
    }

    /* 释放管理器互斥锁 */
    if (manager->manager_mutex != RT_NULL)
    {
        rt_mutex_delete(manager->manager_mutex);
        manager->manager_mutex = RT_NULL;
    }

    /* 断开并释放RS485 */
    if (manager->rs485_handle != RT_NULL)
    {
        rs485_disconn(manager->rs485_handle);
        rs485_destory(manager->rs485_handle);
        manager->rs485_handle = RT_NULL;
    }

    manager->motor_count = 0;
    manager->initialized = RT_FALSE;

    LOG_I("MS4010 manager deinitialized");
    return RT_EOK;
}

/**
 * @brief 添加电机到管理器
 */
rt_err_t ms4010_manager_add_motor(ms4010_manager_t *manager, ms4010_device_t *device, rt_uint8_t motor_id)
{
    RT_ASSERT(manager != RT_NULL);
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(motor_id > 0 && motor_id <= 255);

    if (!manager->initialized)
    {
        LOG_E("Manager not initialized");
        return -MS4010_ERROR;
    }

    /* 获取管理器锁，保护管理器结构体的修改 */
    if (rt_mutex_take(manager->manager_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("Failed to take manager mutex");
        return -MS4010_ERROR;
    }

    /* 检查电机数量限制 */
    if (manager->motor_count >= MS4010_MAX_MOTORS)
    {
        LOG_E("Maximum motor count reached");
        rt_mutex_release(manager->manager_mutex);
        return -MS4010_ERROR;
    }

    /* 检查ID是否已存在 */
    for (int i = 0; i < MS4010_MAX_MOTORS; i++)
    {
        if (manager->motors[i] != RT_NULL && manager->motors[i]->motor_id == motor_id)
        {
            LOG_E("Motor ID %d already exists", motor_id);
            rt_mutex_release(manager->manager_mutex);
            return -MS4010_ERROR;
        }
    }

    /* 清零设备结构体 */
    rt_memset(device, 0, sizeof(ms4010_device_t));

    /* 共享RS485句柄 */
    device->rs485_handle = manager->rs485_handle;

    /* 创建设备专用互斥锁 */
    char mutex_name[16];
    rt_snprintf(mutex_name, sizeof(mutex_name), "ms4010_%d", motor_id);
    device->mutex = rt_mutex_create(mutex_name, RT_IPC_FLAG_FIFO);
    if (device->mutex == RT_NULL)
    {
        LOG_E("Failed to create mutex for motor %d", motor_id);
        rt_mutex_release(manager->manager_mutex);
        return -MS4010_ERROR;
    }

    /* 初始化设备参数 */
    device->motor_id = motor_id;
    device->send_count = 0;
    device->error_count = 0;

    /* 原子性地添加到管理器 */
    for (int i = 0; i < MS4010_MAX_MOTORS; i++)
    {
        if (manager->motors[i] == RT_NULL)
        {
            manager->motors[i] = device;
            manager->motor_count++;
            break;
        }
    }

    rt_mutex_release(manager->manager_mutex);

    LOG_I("MS4010 motor %d added to manager", motor_id);
    return RT_EOK;
}

/**
 * @brief 从管理器移除电机
 */
rt_err_t ms4010_manager_remove_motor(ms4010_manager_t *manager, rt_uint8_t motor_id)
{
    RT_ASSERT(manager != RT_NULL);

    if (!manager->initialized)
    {
        return -MS4010_ERROR;
    }

    /* 获取管理器锁，保护管理器结构体的修改 */
    if (rt_mutex_take(manager->manager_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("Failed to take manager mutex");
        return -MS4010_ERROR;
    }

    for (int i = 0; i < MS4010_MAX_MOTORS; i++)
    {
        if (manager->motors[i] != RT_NULL && manager->motors[i]->motor_id == motor_id)
        {
            ms4010_device_t *device = manager->motors[i];
            
            /* 原子性地从管理器移除 */
            manager->motors[i] = RT_NULL;
            manager->motor_count--;
            
            rt_mutex_release(manager->manager_mutex);
            
            /* 释放设备互斥锁 */
            if (device->mutex != RT_NULL)
            {
                rt_mutex_delete(device->mutex);
                device->mutex = RT_NULL;
            }

            device->rs485_handle = RT_NULL;

            /* 释放动态分配的设备内存 */
            rt_free(device);

            LOG_I("MS4010 motor %d removed from manager", motor_id);
            return RT_EOK;
        }
    }

    rt_mutex_release(manager->manager_mutex);

    LOG_W("Motor ID %d not found in manager", motor_id);
    return -MS4010_ERROR;
}

/**
 * @brief 通过ID获取电机设备
 */
ms4010_device_t* ms4010_manager_get_motor(ms4010_manager_t *manager, rt_uint8_t motor_id)
{
    RT_ASSERT(manager != RT_NULL);

    if (!manager->initialized)
    {
        return RT_NULL;
    }

    ms4010_device_t *result = RT_NULL;

    /* 获取管理器锁，保护管理器结构体的读取 */
    if (rt_mutex_take(manager->manager_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_W("Failed to take manager mutex for reading");
        return RT_NULL;
    }

    for (int i = 0; i < MS4010_MAX_MOTORS; i++)
    {
        if (manager->motors[i] != RT_NULL && manager->motors[i]->motor_id == motor_id)
        {
            result = manager->motors[i];
            break;
        }
    }

    rt_mutex_release(manager->manager_mutex);

    return result;
}

/**
 * @brief 获取所有电机的状态
 */
rt_err_t ms4010_manager_get_all_status(ms4010_manager_t *manager)
{
    RT_ASSERT(manager != RT_NULL);

    if (!manager->initialized)
    {
        return -MS4010_ERROR;
    }

    rt_err_t overall_result = RT_EOK;
    ms4010_device_t *motor_list[MS4010_MAX_MOTORS];
    rt_uint8_t active_count = 0;

    /* 获取管理器锁，复制电机列表以避免长时间锁定 */
    if (rt_mutex_take(manager->manager_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("Failed to take manager mutex");
        return -MS4010_ERROR;
    }

    LOG_I("=== MS4010 Manager Status Report ===");
    LOG_I("Total motors: %d", manager->motor_count);

    /* 快速复制电机指针列表 */
    for (int i = 0; i < MS4010_MAX_MOTORS; i++)
    {
        if (manager->motors[i] != RT_NULL)
        {
            motor_list[active_count++] = manager->motors[i];
        }
    }

    rt_mutex_release(manager->manager_mutex);

    /* 释放管理器锁后进行状态查询，避免长时间锁定 */
    for (int i = 0; i < active_count; i++)
    {
        ms4010_device_t *motor = motor_list[i];
        rt_err_t result = ms4010_read_status1(motor, &motor->status1);
        
        LOG_I("Motor ID %d: Send=%d, Error=%d, Status=%s", 
              motor->motor_id, motor->send_count, motor->error_count,
              (result == RT_EOK) ? "OK" : "FAILED");
        
        if (result == RT_EOK)
        {
            LOG_I("  Temp=%d°C, Voltage=%d.%02dV, Current=%d.%02dA", 
                  motor->status1.temperature,
                  motor->status1.voltage / 100, motor->status1.voltage % 100,
                  motor->status1.current / 100, motor->status1.current % 100);
        }
        else
        {
            overall_result = result;
        }
    }

    return overall_result;
}

/**
 * @brief 自动扫描并添加总线上的所有电机
 */
rt_uint8_t ms4010_manager_auto_scan(ms4010_manager_t *manager, rt_uint8_t start_id, rt_uint8_t end_id, rt_uint32_t timeout_ms)
{
    RT_ASSERT(manager != RT_NULL);
    RT_ASSERT(start_id > 0 && end_id <= 255 && start_id <= end_id);

    if (!manager->initialized)
    {
        LOG_E("Manager not initialized");
        return 0;
    }

    rt_uint8_t found_count = 0;
    rt_uint8_t rx_buffer[MS4010_MAX_FRAME_SIZE];

    LOG_I("=== Starting MS4010 Auto Scan (ID: %d-%d) ===", start_id, end_id);

    /* 临时设置较短的超时时间以加快扫描速度 */
    rs485_set_recv_tmo(manager->rs485_handle, timeout_ms);

    for (rt_uint8_t id = start_id; id <= end_id; id++)
    {
        /* 检查该ID是否已经存在 */
        rt_bool_t already_exists = RT_FALSE;
        for (int i = 0; i < MS4010_MAX_MOTORS; i++)
        {
            if (manager->motors[i] != RT_NULL && manager->motors[i]->motor_id == id)
            {
                already_exists = RT_TRUE;
                break;
            }
        }

        if (already_exists)
        {
            LOG_D("Motor ID %d already exists, skipping", id);
            continue;
        }

        /* 尝试与该ID通信 */
        rt_err_t result = ms4010_send_command_shared(
            manager->rs485_handle, 
            manager->bus_mutex,
            id,
            MS4010_CMD_READ_STATUS1,
            0,
            RT_NULL,
            MS4010_RESPONSE_SIZE_STATUS1,
            rx_buffer
        );

        if (result == RT_EOK)
        {
            /* 发现电机，尝试添加到管理器 */
            LOG_I("Found motor at ID %d", id);
            
            /* 查找空闲的设备槽位 */
            ms4010_device_t *device = RT_NULL;
            int slot_index = -1;
            
            /* 动态分配电机设备内存 */
            device = (ms4010_device_t*)rt_malloc(sizeof(ms4010_device_t));
            if (device == RT_NULL)
            {
                LOG_E("Failed to allocate memory for motor ID %d", id);
                continue;
            }

            /* 添加到管理器 */
            rt_err_t add_result = ms4010_manager_add_motor(manager, device, id);
            if (add_result == RT_EOK)
            {
                found_count++;
                LOG_I("Successfully added motor ID %d (%d/%d)", id, found_count, manager->motor_count);
            }
            else
            {
                LOG_W("Failed to add motor ID %d: %d", id, add_result);
                rt_free(device);
            }
        }
        else
        {
            LOG_D("No response from ID %d", id);
        }

        /* 短暂延时避免总线冲突 */
        rt_thread_mdelay(10);
    }

    /* 恢复默认超时时间 */
    rs485_set_recv_tmo(manager->rs485_handle, MS4010_DEFAULT_TIMEOUT);

    LOG_I("=== Auto Scan Complete: Found %d motors ===", found_count);
    return found_count;
}

/**
 * @brief 快速扫描常用ID范围的电机
 */
rt_uint8_t ms4010_manager_quick_scan(ms4010_manager_t *manager)
{
    RT_ASSERT(manager != RT_NULL);

    if (!manager->initialized)
    {
        LOG_E("Manager not initialized");
        return 0;
    }

    rt_uint8_t total_found = 0;
    rt_uint8_t found_count;

    LOG_I("=== Starting MS4010 Quick Scan ===");

    /* 扫描常用ID范围：1-16 (大多数应用场景) */
    LOG_I("Scanning common range 1-2...");
    found_count = ms4010_manager_auto_scan(manager, 1, 2, 50); // 50ms超时
    total_found += found_count;

    /* 如果在常用范围内没找到足够的电机，扫描扩展范围 */
    // if (found_count < 2)
    // {
    //     LOG_I("Scanning extended range 17-32...");
    //     found_count = ms4010_manager_auto_scan(manager, 2, 32, 50);
    //     total_found += found_count;
    // }

    // /* 如果还是没找到，扫描更大范围 */
    // if (total_found == 0)
    // {
    //     LOG_I("Scanning wider range 33-64...");
    //     found_count = ms4010_manager_auto_scan(manager, 33, 64, 30); // 更短超时
    //     total_found += found_count;
    // }

    LOG_I("=== Quick Scan Complete: Total %d motors found ===", total_found);
    return total_found;
}

/**
 * @brief 清除所有自动发现的电机
 */
rt_err_t ms4010_manager_clear_all(ms4010_manager_t *manager)
{
    RT_ASSERT(manager != RT_NULL);

    if (!manager->initialized)
    {
        return -MS4010_ERROR;
    }

    rt_uint8_t removed_count = 0;
    ms4010_device_t *devices_to_free[MS4010_MAX_MOTORS];
    rt_uint8_t free_count = 0;

    LOG_I("=== Clearing all motors from manager ===");

    /* 获取管理器锁，原子性地清除所有电机 */
    if (rt_mutex_take(manager->manager_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("Failed to take manager mutex");
        return -MS4010_ERROR;
    }

    /* 收集需要释放的设备指针 */
    for (int i = 0; i < MS4010_MAX_MOTORS; i++)
    {
        if (manager->motors[i] != RT_NULL)
        {
            devices_to_free[free_count++] = manager->motors[i];
            manager->motors[i] = RT_NULL;
            removed_count++;
        }
    }

    manager->motor_count = 0;

    rt_mutex_release(manager->manager_mutex);

    /* 释放管理器锁后，清理设备资源 */
    for (int i = 0; i < free_count; i++)
    {
        ms4010_device_t *device = devices_to_free[i];
        rt_uint8_t motor_id = device->motor_id;
        
        /* 释放设备互斥锁 */
        if (device->mutex != RT_NULL)
        {
            rt_mutex_delete(device->mutex);
            device->mutex = RT_NULL;
        }
        
        device->rs485_handle = RT_NULL;
        
        /* 释放动态分配的设备内存 */
        rt_free(device);
        
        LOG_I("Removed motor ID %d", motor_id);
    }

    LOG_I("=== Clear Complete: Removed %d motors ===", removed_count);
    return RT_EOK;
}

/**
 * @brief 初始化MS4010电机驱动
 */
rt_err_t ms4010_init(ms4010_device_t *device, const char *rs485_name, rt_uint8_t motor_id, rt_uint32_t baudrate)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(rs485_name != RT_NULL);
    RT_ASSERT(motor_id > 0 && motor_id <= 255);

    /* 清零设备结构体 */
    rt_memset(device, 0, sizeof(ms4010_device_t));

    /* 创建RS485实例 */
    device->rs485_handle = rs485_create(rs485_name, baudrate, RS485_SAMPLE_MASTER_PARITY, RS485_SAMPLE_MASTER_PIN, RS485_SAMPLE_MASTER_LVL);
    if (device->rs485_handle == RT_NULL)
    {
        LOG_E("Failed to create RS485 instance for %s", rs485_name);
        return -MS4010_ERROR;
    }
		
		//修改波特率
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
		
    config.baud_rate = baudrate;
    config.data_bits = DATA_BITS_8;
    config.parity = PARITY_NONE;
    config.stop_bits =STOP_BITS_1;
    config.tx_bufsz= BSP_UART2_TX_BUFSIZE;
		config.rx_bufsz=BSP_UART2_RX_BUFSIZE;
		LOG_E("USing uart2 TX RX buffer");
		//rt_device_control(hinst->serial, RT_DEVICE_CTRL_CONFIG, &config);
		if(rt_device_control(rt_device_find(USART_NAME), RT_DEVICE_CTRL_CONFIG, &config)!=RT_EOK)
		{
			LOG_E("%s Buad Change Error",USART_NAME);
		}
		else{
			LOG_E("%s Buad Change OK",USART_NAME);
		}
		//修改波特率
		

    /* 设置超时时间 */
    rs485_set_recv_tmo(device->rs485_handle, MS4010_DEFAULT_TIMEOUT);

    /* 连接RS485 */
    if (rs485_connect(device->rs485_handle) != RT_EOK)
    {
        LOG_E("Failed to connect RS485");
        rs485_destory(device->rs485_handle);
        return -MS4010_ERROR;
    }

    /* 创建互斥锁 */
    device->mutex = rt_mutex_create("ms4010_mutex", RT_IPC_FLAG_FIFO);
    if (device->mutex == RT_NULL)
    {
        LOG_E("Failed to create mutex");
        rs485_disconn(device->rs485_handle);
        rs485_destory(device->rs485_handle);
        return -MS4010_ERROR;
    }

    /* 初始化设备参数 */
    device->motor_id = motor_id;
    device->send_count = 0;
    device->error_count = 0;

    LOG_I("MS4010 motor %d initialized successfully on %s", motor_id, rs485_name);
    return RT_EOK;
}

/**
 * @brief 反初始化MS4010电机驱动
 */
rt_err_t ms4010_deinit(ms4010_device_t *device)
{
    RT_ASSERT(device != RT_NULL);

    if (device->mutex != RT_NULL)
    {
        rt_mutex_delete(device->mutex);
        device->mutex = RT_NULL;
    }

    if (device->rs485_handle != RT_NULL)
    {
        rs485_disconn(device->rs485_handle);
        rs485_destory(device->rs485_handle);
        device->rs485_handle = RT_NULL;
    }

    LOG_I("MS4010 motor %d deinitialized", device->motor_id);
    return RT_EOK;
}

/**
 * @brief 开环控制
 */
rt_err_t ms4010_open_loop_control(ms4010_device_t *device, rt_int16_t value)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[2];
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_OPEN_CONTROL, 2, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 力矩控制
 */
rt_err_t ms4010_torque_control(ms4010_device_t *device, rt_int16_t torque)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[2];
    data[0] = torque & 0xFF;
    data[1] = (torque >> 8) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_TORQUE_CONTROL, 2, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 速度控制
 */
rt_err_t ms4010_speed_control(ms4010_device_t *device, rt_int32_t speed)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[4];
    data[0] = speed & 0xFF;
    data[1] = (speed >> 8) & 0xFF;
    data[2] = (speed >> 16) & 0xFF;
    data[3] = (speed >> 24) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_SPEED_CONTROL, 4, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 位置控制(原版，保持兼容)
 */
rt_err_t ms4010_position_control(ms4010_device_t *device, rt_int32_t angle, rt_uint16_t speed)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[8];
    rt_int64_t angle_64 = (rt_int64_t)angle;
    
    data[0] = angle_64 & 0xFF;
    data[1] = (angle_64 >> 8) & 0xFF;
    data[2] = (angle_64 >> 16) & 0xFF;
    data[3] = (angle_64 >> 24) & 0xFF;
    data[4] = (angle_64 >> 32) & 0xFF;
    data[5] = (angle_64 >> 40) & 0xFF;
    data[6] = (angle_64 >> 48) & 0xFF;
    data[7] = (angle_64 >> 56) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_ANGLE_CONTROL, 8, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 读取电机详细状态1(温度、电压、电流、状态)
 */
rt_err_t ms4010_read_status1(ms4010_device_t *device, ms4010_status1_t *status1)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(status1 != RT_NULL);

    rt_err_t result = ms4010_send_command(device, MS4010_CMD_READ_STATUS1, 0, RT_NULL, MS4010_RESPONSE_SIZE_STATUS1);
    
    if (result == RT_EOK && rt_mutex_take(device->mutex, RT_WAITING_FOREVER) == RT_EOK)
    {
        rt_memcpy(status1, &device->status1, sizeof(ms4010_status1_t));
        rt_mutex_release(device->mutex);
    }
    
    return result;
}

/**
 * @brief 清除电机错误标志
 */
rt_err_t ms4010_clear_error(ms4010_device_t *device)
{
    RT_ASSERT(device != RT_NULL);
    
    return ms4010_send_command(device, MS4010_CMD_CLEAR_ERROR, 0, RT_NULL, MS4010_RESPONSE_SIZE_STATUS1);
}

/**
 * @brief 读取电机相电流状态3
 */
rt_err_t ms4010_read_status3(ms4010_device_t *device, ms4010_status3_t *status3)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(status3 != RT_NULL);

    rt_err_t result = ms4010_send_command(device, MS4010_CMD_READ_STATUS3, 0, RT_NULL, MS4010_RESPONSE_SIZE_STATUS3);
    
    if (result == RT_EOK && rt_mutex_take(device->mutex, RT_WAITING_FOREVER) == RT_EOK)
    {
        rt_memcpy(status3, &device->status3, sizeof(ms4010_status3_t));
        rt_mutex_release(device->mutex);
    }
    
    return result;
}

/**
 * @brief 电机开启命令
 */
rt_err_t ms4010_motor_on(ms4010_device_t *device)
{
    RT_ASSERT(device != RT_NULL);
    
    return ms4010_send_command(device, MS4010_CMD_MOTOR_ON, 0, RT_NULL, MS4010_RESPONSE_SIZE_SIMPLE);
}

/**
 * @brief 电机关闭命令
 */
rt_err_t ms4010_motor_off(ms4010_device_t *device)
{
    RT_ASSERT(device != RT_NULL);
    
    return ms4010_send_command(device, MS4010_CMD_MOTOR_OFF, 0, RT_NULL, MS4010_RESPONSE_SIZE_SIMPLE);
}

/**
 * @brief 电机停止命令
 */
rt_err_t ms4010_motor_stop(ms4010_device_t *device)
{
    RT_ASSERT(device != RT_NULL);
    
    return ms4010_send_command(device, MS4010_CMD_MOTOR_STOP, 0, RT_NULL, MS4010_RESPONSE_SIZE_SIMPLE);
}

/**
 * @brief 抱闸控制
 */
rt_err_t ms4010_brake_control(ms4010_device_t *device, rt_uint8_t brake_cmd, rt_uint8_t *brake_status)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[1] = {brake_cmd};
    rt_err_t result = ms4010_send_command(device, MS4010_CMD_BRAKE_CONTROL, 1, data, 7); // 6byte data + 1byte checksum
    
    if (result == RT_EOK && brake_status != RT_NULL && brake_cmd == MS4010_BRAKE_READ_STATUS)
    {
        // 返回抱闸状态，需要从响应中提取
        if (rt_mutex_take(device->mutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            *brake_status = device->rx_buffer[5]; // DATA[0]位置
            rt_mutex_release(device->mutex);
        }
    }
    
    return result;
}

/**
 * @brief 设置当前位置为任意角度(写入RAM)
 */
rt_err_t ms4010_set_current_position(ms4010_device_t *device, rt_int32_t angle)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[4];
    // 多圈角度 (4字节)
    data[0] = angle & 0xFF;
    data[1] = (angle >> 8) & 0xFF;
    data[2] = (angle >> 16) & 0xFF;
    data[3] = (angle >> 24) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_SET_CURRENT_POSITION, 4, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 位置控制(多圈，带速度限制)
 */
rt_err_t ms4010_position_control2(ms4010_device_t *device, rt_int64_t angle, rt_uint32_t max_speed)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[12];
    // 角度 (8字节)
    data[0] = angle & 0xFF;
    data[1] = (angle >> 8) & 0xFF;
    data[2] = (angle >> 16) & 0xFF;
    data[3] = (angle >> 24) & 0xFF;
    data[4] = (angle >> 32) & 0xFF;
    data[5] = (angle >> 40) & 0xFF;
    data[6] = (angle >> 48) & 0xFF;
    data[7] = (angle >> 56) & 0xFF;
    // 最大速度 (4字节)
    data[8] = max_speed & 0xFF;
    data[9] = (max_speed >> 8) & 0xFF;
    data[10] = (max_speed >> 16) & 0xFF;
    data[11] = (max_speed >> 24) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_ANGLE_CONTROL2, 12, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 单圈位置控制1
 */
rt_err_t ms4010_single_position_control1(ms4010_device_t *device, rt_uint8_t direction, rt_uint16_t angle)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(angle <= 35999); // 0~359.99度
    
    rt_uint8_t data[4];
    data[0] = direction;
    data[1] = angle & 0xFF;
    data[2] = (angle >> 8) & 0xFF;
    data[3] = 0x00; // NULL
    
    return ms4010_send_command(device, MS4010_CMD_SINGLE_ANGLE_CONTROL1, 4, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 单圈位置控制2(带速度限制)
 */
rt_err_t ms4010_single_position_control2(ms4010_device_t *device, rt_uint8_t direction, rt_uint16_t angle, rt_uint32_t max_speed)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(angle <= 35999); // 0~359.99度
    
    rt_uint8_t data[8];
    data[0] = direction;
    data[1] = angle & 0xFF;
    data[2] = (angle >> 8) & 0xFF;
    data[3] = 0x00; // NULL
    // 最大速度 (4字节)
    data[4] = max_speed & 0xFF;
    data[5] = (max_speed >> 8) & 0xFF;
    data[6] = (max_speed >> 16) & 0xFF;
    data[7] = (max_speed >> 24) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_SINGLE_ANGLE_CONTROL2, 8, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 增量位置控制1
 */
rt_err_t ms4010_increment_position_control1(ms4010_device_t *device, rt_int32_t angle_increment)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[4];
    data[0] = angle_increment & 0xFF;
    data[1] = (angle_increment >> 8) & 0xFF;
    data[2] = (angle_increment >> 16) & 0xFF;
    data[3] = (angle_increment >> 24) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_INCREMENT_CONTROL1, 4, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 增量位置控制2(带速度限制)
 */
rt_err_t ms4010_increment_position_control2(ms4010_device_t *device, rt_int32_t angle_increment, rt_uint32_t max_speed)
{
    RT_ASSERT(device != RT_NULL);
    
    rt_uint8_t data[8];
    // 角度增量 (4字节)
    data[0] = angle_increment & 0xFF;
    data[1] = (angle_increment >> 8) & 0xFF;
    data[2] = (angle_increment >> 16) & 0xFF;
    data[3] = (angle_increment >> 24) & 0xFF;
    // 最大速度 (4字节)
    data[4] = max_speed & 0xFF;
    data[5] = (max_speed >> 8) & 0xFF;
    data[6] = (max_speed >> 16) & 0xFF;
    data[7] = (max_speed >> 24) & 0xFF;
    
    return ms4010_send_command(device, MS4010_CMD_INCREMENT_CONTROL2, 8, data, MS4010_RESPONSE_SIZE_CONTROL);
}

/**
 * @brief 获取电机状态(兼容原版)
 */
rt_err_t ms4010_get_status(ms4010_device_t *device, ms4010_status_t *status)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(status != RT_NULL);

    rt_err_t result = ms4010_send_command(device, MS4010_CMD_READ_STATUS2, 0, RT_NULL, MS4010_RESPONSE_SIZE_STATUS2);
    
    if (result == RT_EOK && rt_mutex_take(device->mutex, RT_WAITING_FOREVER) == RT_EOK)
    {
        rt_memcpy(status, &device->status, sizeof(ms4010_status_t));
        rt_mutex_release(device->mutex);
    }
    
    return result;
}

/**
 * @brief 获取统计信息
 */
rt_err_t ms4010_get_statistics(ms4010_device_t *device, rt_uint32_t *send_count, rt_uint32_t *error_count)
{
    RT_ASSERT(device != RT_NULL);

    if (send_count != RT_NULL)
        *send_count = device->send_count;
    
    if (error_count != RT_NULL)
        *error_count = device->error_count;

    return RT_EOK;
}

/**
 * @brief 发送命令到MS4010电机 (共享总线版本)
 */
static rt_err_t ms4010_send_command_shared(rs485_inst_t *rs485_handle, rt_mutex_t bus_mutex, rt_uint8_t motor_id, rt_uint8_t cmd, rt_uint8_t data_size, const rt_uint8_t *data, rt_uint8_t expected_response_size, rt_uint8_t *rx_buffer)
{
    rt_err_t result = RT_EOK;
    rt_uint8_t tx_buffer[MS4010_MAX_FRAME_SIZE];
    rt_uint8_t frame_size = 0;
    rt_int32_t recv_len = 0;

    if (rt_mutex_take(bus_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -MS4010_ERROR;
    }

    /* 打包命令帧 */
    result = ms4010_pack_frame(tx_buffer, cmd, motor_id, data_size, data, &frame_size);
    if (result != RT_EOK)
    {
        rt_mutex_release(bus_mutex);
        return result;
    }

    /* 发送命令并接收响应 */
    recv_len = rs485_send_then_recv(rs485_handle, 
                                   tx_buffer, frame_size,
                                   rx_buffer, expected_response_size);

    rt_mutex_release(bus_mutex);

    if (recv_len != expected_response_size)
    {
        LOG_W("MS4010 motor %d: cmd is 0x%x Expected %d bytes, received %d bytes", 
              motor_id, cmd, expected_response_size, recv_len);
        
        /* 清空接收缓冲区防止影响后续命令 */
        ms4010_clear_rx_buffer(rs485_handle);
        
        result = -MS4010_ETIMEOUT;
    }
    else
    {
        /* 基本的帧头检查 */
        if (rx_buffer[0] != MS4010_FRAME_HEAD)
        {
            LOG_W("MS4010 motor %d: Invalid response frame header 0x%02X", motor_id, rx_buffer[0]);
            
            /* 清空接收缓冲区防止影响后续命令 */
            ms4010_clear_rx_buffer(rs485_handle);
            
            result = -MS4010_EFRAME;
        }
    }

    return result;
}

/**
 * @brief 发送命令到MS4010电机
 */
static rt_err_t ms4010_send_command(ms4010_device_t *device, rt_uint8_t cmd, rt_uint8_t data_size, const rt_uint8_t *data, rt_uint8_t expected_response_size)
{
    rt_err_t result = RT_EOK;
    rt_uint8_t frame_size = 0;
    rt_int32_t recv_len = 0;

    if (rt_mutex_take(device->mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -MS4010_ERROR;
    }

    /* 打包命令帧 */
    result = ms4010_pack_frame(device->tx_buffer, cmd, device->motor_id, data_size, data, &frame_size);
    if (result != RT_EOK)
    {
        device->error_count++;
        rt_mutex_release(device->mutex);
        return result;
    }

    /* 发送命令并接收响应 */
//		rt_enter_critical();
    recv_len = rs485_send_then_recv(device->rs485_handle, 
                                   device->tx_buffer, frame_size,
                                   device->rx_buffer, expected_response_size);
//    rt_exit_critical();
    device->send_count++;

    if (recv_len != expected_response_size)
    {
        LOG_W("MS4010 motor %d: cmd is 0x%x Expected %d bytes, received %d bytes", 
              device->motor_id, cmd, expected_response_size, recv_len);
        device->error_count++;
        
        /* 清空接收缓冲区防止影响后续命令 */
        ms4010_clear_rx_buffer(device->rs485_handle);
        
        result = -MS4010_ETIMEOUT;
    }
    else
    {
        /* 根据命令类型解析响应 */
        switch (cmd)
        {
            case MS4010_CMD_READ_STATUS1:
            case MS4010_CMD_CLEAR_ERROR:
                result = ms4010_parse_status1_response(device, device->rx_buffer, recv_len);
                break;
                
            case MS4010_CMD_READ_STATUS3:
                result = ms4010_parse_status3_response(device, device->rx_buffer, recv_len);
                break;
                
            case MS4010_CMD_READ_STATUS2:
            case MS4010_CMD_OPEN_CONTROL:
            case MS4010_CMD_TORQUE_CONTROL:
            case MS4010_CMD_SPEED_CONTROL:
            case MS4010_CMD_ANGLE_CONTROL:
            case MS4010_CMD_ANGLE_CONTROL2:
            case MS4010_CMD_SINGLE_ANGLE_CONTROL1:
            case MS4010_CMD_SINGLE_ANGLE_CONTROL2:
            case MS4010_CMD_INCREMENT_CONTROL1:
            case MS4010_CMD_INCREMENT_CONTROL2:
                result = ms4010_parse_response(device, device->rx_buffer, recv_len);
                break;
                
            default:
                // 简单命令，只检查帧头和校验
                if (device->rx_buffer[0] != MS4010_FRAME_HEAD)
                {
                    LOG_W("MS4010 motor %d: Invalid response frame header 0x%02X", device->motor_id, device->rx_buffer[0]);
                    result = -MS4010_EFRAME;
                }
                break;
        }
        
        if (result != RT_EOK)
        {
            device->error_count++;
            /* 当解析错误时，清空接收缓冲区防止影响后续命令 */
            ms4010_clear_rx_buffer(device->rs485_handle);
        }
    }

    rt_mutex_release(device->mutex);
    return result;
}

/**
 * @brief 打包MS4010命令帧
 */
static rt_err_t ms4010_pack_frame(rt_uint8_t *buffer, rt_uint8_t cmd, rt_uint8_t id, rt_uint8_t data_size, const rt_uint8_t *data, rt_uint8_t *frame_size)
{
    RT_ASSERT(buffer != RT_NULL);
    RT_ASSERT(frame_size != RT_NULL);

    if (data_size > 0 && data == RT_NULL)
    {
        return -MS4010_EINVAL;
    }

    /* 构造帧头 */
    buffer[0] = MS4010_FRAME_HEAD;  // 帧头
    buffer[1] = cmd;                // 命令
    buffer[2] = id;                 // 电机ID
    buffer[3] = data_size;          // 数据长度

    /* 计算帧头校验和 */
    buffer[4] = ms4010_calculate_checksum(buffer, 4);

    /* 复制数据 */
    if (data_size > 0)
    {
        rt_memcpy(&buffer[MS4010_LEAST_FRAME_SIZE], data, data_size);
        /* 计算数据校验和 */
        buffer[MS4010_LEAST_FRAME_SIZE + data_size] = ms4010_calculate_checksum(&buffer[MS4010_LEAST_FRAME_SIZE], data_size);
        *frame_size = MS4010_LEAST_FRAME_SIZE + data_size + 1;
    }
    else
    {
        *frame_size = MS4010_LEAST_FRAME_SIZE;
    }

    return RT_EOK;
}

/**
 * @brief 解析MS4010响应帧
 */
static rt_err_t ms4010_parse_response(ms4010_device_t *device, const rt_uint8_t *buffer, rt_uint8_t size)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if (size != MS4010_RESPONSE_SIZE)
    {
        return -MS4010_EFRAME;
    }

    /* 检查帧头 */
    if (buffer[0] != MS4010_FRAME_HEAD)
    {
        LOG_W("MS4010 motor %d: Invalid frame header 0x%02X", device->motor_id, buffer[0]);
        return -MS4010_EFRAME;
    }

    /* 检查帧头校验和 */
    rt_uint8_t head_checksum = ms4010_calculate_checksum(buffer, 4);
    if (buffer[4] != head_checksum)
    {
        LOG_W("MS4010 motor %d: Header checksum error, expected 0x%02X, got 0x%02X", 
              device->motor_id, head_checksum, buffer[4]);
        return -MS4010_ECHECKSUM;
    }

    /* 检查数据校验和 */
    rt_uint8_t data_checksum = ms4010_calculate_checksum(&buffer[5], 7);
    if (buffer[12] != data_checksum)
    {
        LOG_W("MS4010 motor %d: Data checksum error, expected 0x%02X, got 0x%02X", 
              device->motor_id, data_checksum, buffer[12]);
        return -MS4010_ECHECKSUM;
    }

    /* 解析状态数据 */
    device->status.temperature = (rt_int8_t)buffer[5];
    device->status.power_or_torque = (rt_int16_t)(buffer[6] | (buffer[7] << 8));
    device->status.speed = (rt_int16_t)(buffer[8] | (buffer[9] << 8));
    device->status.encoder = (rt_uint16_t)(buffer[10] | (buffer[11] << 8));
    device->status.last_update_time = rt_tick_get();

    return RT_EOK;
}

/**
 * @brief 清空RS485接收缓冲区
 * @param rs485_handle RS485句柄
 * @return rt_err_t 错误码
 */
static rt_err_t ms4010_clear_rx_buffer(rs485_inst_t *rs485_handle)
{
    RT_ASSERT(rs485_handle != RT_NULL);
    
    rt_uint8_t dummy_buffer[64];  // 临时缓冲区
    rt_int32_t recv_len;
    rt_uint32_t clear_count = 0;
    rt_uint32_t max_clear_attempts = 10; // 最大清空尝试次数
    
    LOG_D("Clearing RS485 receive buffer...");
    
    /* 中断当前的接收等待 */
    rs485_break_recv(rs485_handle);
    
    /* 短暂延时让总线稳定 */
    rt_thread_mdelay(5);
    
    /* 循环读取剩余数据直到缓冲区为空 */
    while (clear_count < max_clear_attempts)
    {
        /* 尝试以最短超时读取数据 */
        int original_timeout = 0;
        rs485_set_recv_tmo(rs485_handle, 10); // 设置10ms超时
        
        recv_len = rs485_recv(rs485_handle, dummy_buffer, sizeof(dummy_buffer));
        
        if (recv_len <= 0)
        {
            /* 没有更多数据，缓冲区已清空 */
            break;
        }
        
        clear_count++;
        LOG_D("Cleared %d bytes from buffer (attempt %d)", recv_len, clear_count);
    }
    
    /* 恢复原始超时设置 */
    rs485_set_recv_tmo(rs485_handle, MS4010_DEFAULT_TIMEOUT);
    
    if (clear_count > 0)
    {
        LOG_I("RS485 buffer cleared after %d attempts", clear_count);
    }
    
    return RT_EOK;
}

/**
 * @brief 计算校验和
 */
static rt_uint8_t ms4010_calculate_checksum(const rt_uint8_t *buffer, rt_uint8_t size)
{
    rt_uint8_t checksum = 0;
    for (rt_uint8_t i = 0; i < size; i++)
    {
        checksum += buffer[i];
    }
    return checksum;
}

/**
 * @brief 解析MS4010状态1响应帧
 */
static rt_err_t ms4010_parse_status1_response(ms4010_device_t *device, const rt_uint8_t *buffer, rt_uint8_t size)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if (size != MS4010_RESPONSE_SIZE_STATUS1)
    {
        return -MS4010_EFRAME;
    }

    /* 检查帧头 */
    if (buffer[0] != MS4010_FRAME_HEAD)
    {
        LOG_W("MS4010 motor %d: Invalid frame header 0x%02X", device->motor_id, buffer[0]);
        return -MS4010_EFRAME;
    }

    /* 检查帧头校验和 */
    rt_uint8_t head_checksum = ms4010_calculate_checksum(buffer, 4);
    if (buffer[4] != head_checksum)
    {
        LOG_W("MS4010 motor %d: Header checksum error, expected 0x%02X, got 0x%02X", 
              device->motor_id, head_checksum, buffer[4]);
        return -MS4010_ECHECKSUM;
    }

    /* 检查数据校验和 */
    rt_uint8_t data_checksum = ms4010_calculate_checksum(&buffer[5], 7);
    if (buffer[12] != data_checksum)
    {
        LOG_W("MS4010 motor %d: Data checksum error, expected 0x%02X, got 0x%02X", 
              device->motor_id, data_checksum, buffer[12]);
        return -MS4010_ECHECKSUM;
    }

    /* 解析状态1数据 */
    device->status1.temperature = (rt_int8_t)buffer[5];
    device->status1.voltage = (rt_int16_t)(buffer[6] | (buffer[7] << 8));
    device->status1.current = (rt_int16_t)(buffer[8] | (buffer[9] << 8));
    device->status1.motor_state = buffer[10];
    device->status1.error_state = buffer[11];
    device->status1.last_update_time = rt_tick_get();

    return RT_EOK;
}

/**
 * @brief 解析MS4010状态3响应帧
 */
static rt_err_t ms4010_parse_status3_response(ms4010_device_t *device, const rt_uint8_t *buffer, rt_uint8_t size)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if (size != MS4010_RESPONSE_SIZE_STATUS3)
    {
        return -MS4010_EFRAME;
    }

    /* 检查帧头 */
    if (buffer[0] != MS4010_FRAME_HEAD)
    {
        LOG_W("MS4010 motor %d: Invalid frame header 0x%02X", device->motor_id, buffer[0]);
        return -MS4010_EFRAME;
    }

    /* 检查帧头校验和 */
    rt_uint8_t head_checksum = ms4010_calculate_checksum(buffer, 4);
    if (buffer[4] != head_checksum)
    {
        LOG_W("MS4010 motor %d: Header checksum error, expected 0x%02X, got 0x%02X", 
              device->motor_id, head_checksum, buffer[4]);
        return -MS4010_ECHECKSUM;
    }

    /* 检查数据校验和 */
    rt_uint8_t data_checksum = ms4010_calculate_checksum(&buffer[5], 7);
    if (buffer[12] != data_checksum)
    {
        LOG_W("MS4010 motor %d: Data checksum error, expected 0x%02X, got 0x%02X", 
              device->motor_id, data_checksum, buffer[12]);
        return -MS4010_ECHECKSUM;
    }

    /* 解析状态3数据 - 根据协议文档修正索引 */
    device->status3.temperature = (rt_int8_t)buffer[5];      // DATA[0] = 温度
    device->status3.current_a = (rt_int16_t)(buffer[6] | (buffer[7] << 8));   // DATA[1,2] = A相电流
    device->status3.current_b = (rt_int16_t)(buffer[8] | (buffer[9] << 8));   // DATA[3,4] = B相电流
    device->status3.current_c = (rt_int16_t)(buffer[10] | (buffer[11] << 8)); // DATA[5,6] = C相电流
    device->status3.last_update_time = rt_tick_get();

    return RT_EOK;
}


