/**
 * @file drv_ms4010.h
 * @brief MS4010电机RS485驱动头文件
 * @version 1.0
 * @date 2025-07-25
 * @author Your Name
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 本文件定义了MS4010电机的RS485驱动接口和相关数据结构
 * 基于RT-Thread的RS485包进行封装
 */

#ifndef __DRV_MS4010_H__
#define __DRV_MS4010_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 包含必要的头文件 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <rs485.h>

/* MS4010协议定义 */
#define MS4010_FRAME_HEAD                   0x3E    // 帧头
#define MS4010_LEAST_FRAME_SIZE             5       // 最小帧长度 header + cmd + id + dataLength + headCheckSum

/* MS4010命令定义 */
#define MS4010_CMD_READ_STATUS1             0x9A    // 读取状态1和错误标志
#define MS4010_CMD_CLEAR_ERROR              0x9B    // 清除错误标志
#define MS4010_CMD_READ_STATUS2             0x9C    // 读取状态2
#define MS4010_CMD_READ_STATUS3             0x9D    // 读取状态3(相电流)
#define MS4010_CMD_MOTOR_OFF                0x80    // 电机关闭
#define MS4010_CMD_MOTOR_ON                 0x88    // 电机开启
#define MS4010_CMD_MOTOR_STOP               0x81    // 电机停止
#define MS4010_CMD_BRAKE_CONTROL            0x8C    // 抱闸控制
#define MS4010_CMD_OPEN_CONTROL             0xA0    // 开环控制
#define MS4010_CMD_TORQUE_CONTROL           0xA1    // 力矩控制
#define MS4010_CMD_SPEED_CONTROL            0xA2    // 速度控制
#define MS4010_CMD_ANGLE_CONTROL            0xA3    // 位置控制(多圈1)
#define MS4010_CMD_ANGLE_CONTROL2           0xA4    // 位置控制(多圈2)
#define MS4010_CMD_SINGLE_ANGLE_CONTROL1    0xA5    // 单圈位置控制1
#define MS4010_CMD_SINGLE_ANGLE_CONTROL2    0xA6    // 单圈位置控制2
#define MS4010_CMD_INCREMENT_CONTROL1       0xA7    // 增量位置控制1
#define MS4010_CMD_INCREMENT_CONTROL2       0xA8    // 增量位置控制2
#define MS4010_CMD_SET_CURRENT_POSITION     0x95    // 设置当前位置为任意角度

/* MS4010配置参数 */
#define MS4010_DEFAULT_BAUDRATE             115200  // 默认波特率
#define MS4010_DEFAULT_TIMEOUT              5    // 默认超时时间(ms)
#define MS4010_MAX_FRAME_SIZE               32      // 最大帧长度
#define MS4010_MAX_MOTORS                   16      // 支持的最大电机数量
#define MS4010_RESPONSE_SIZE_STATUS1        13      // 状态1响应长度
#define MS4010_RESPONSE_SIZE_STATUS2        13      // 状态2响应长度  
#define MS4010_RESPONSE_SIZE_STATUS3        13      // 状态3响应长度
#define MS4010_RESPONSE_SIZE_CONTROL        13      // 控制命令响应长度
#define MS4010_RESPONSE_SIZE_SIMPLE         5       // 简单命令响应长度
#define MS4010_RESPONSE_SIZE                13
/* 抱闸控制参数 */
#define MS4010_BRAKE_RELEASE                0x01    // 抱闸释放
#define MS4010_BRAKE_LOCK                   0x00    // 抱闸锁定
#define MS4010_BRAKE_READ_STATUS            0x10    // 读取抱闸状态

/* 电机状态位定义 */
#define MS4010_MOTOR_STATE_ON               0x00    // 电机开启状态
#define MS4010_MOTOR_STATE_OFF              0x10    // 电机关闭状态

/* 错误状态位定义 */
#define MS4010_ERROR_LOW_VOLTAGE            (1<<0)  // 低电压
#define MS4010_ERROR_HIGH_VOLTAGE           (1<<1)  // 高电压
#define MS4010_ERROR_DRIVER_TEMP            (1<<2)  // 驱动过温
#define MS4010_ERROR_MOTOR_TEMP             (1<<3)  // 电机过温
#define MS4010_ERROR_OVER_CURRENT           (1<<4)  // 过流
#define MS4010_ERROR_SHORT_CIRCUIT          (1<<5)  // 短路
#define MS4010_ERROR_STALL                  (1<<6)  // 堵转
#define MS4010_ERROR_INPUT_SIGNAL           (1<<7)  // 输入信号丢失

/* MS4010错误码 */
#define MS4010_EOK                          0       // 成功
#define MS4010_ERROR                        -1      // 通用错误
#define MS4010_ETIMEOUT                     -2      // 超时
#define MS4010_EINVAL                       -3      // 参数错误
#define MS4010_EFRAME                       -4      // 帧格式错误
#define MS4010_ECHECKSUM                    -5      // 校验和错误

/* MS4010控制模式枚举 */
typedef enum
{
    MS4010_MODE_OPEN_LOOP = 0,          // 开环控制模式
    MS4010_MODE_TORQUE,                 // 力矩控制模式
    MS4010_MODE_SPEED,                  // 速度控制模式
    MS4010_MODE_POSITION                // 位置控制模式
} ms4010_control_mode_t;

/* MS4010电机状态结构体 */
typedef struct
{
    rt_int8_t   temperature;            // 电机温度
    rt_int16_t  power_or_torque;        // 功率或力矩
    rt_int16_t  speed;                  // 转速
    rt_uint16_t encoder;                // 编码器值
    rt_uint32_t last_update_time;       // 最后更新时间
} ms4010_status_t;

/* MS4010详细状态结构体(状态1) */
typedef struct
{
    rt_int8_t   temperature;            // 电机温度 (°C)
    rt_int16_t  voltage;                // 母线电压 (0.01V/LSB)
    rt_int16_t  current;                // 母线电流 (0.01A/LSB)
    rt_uint8_t  motor_state;            // 电机状态
    rt_uint8_t  error_state;            // 错误状态
    rt_uint32_t last_update_time;       // 最后更新时间
} ms4010_status1_t;

/* MS4010相电流状态结构体(状态3) */
typedef struct
{
    rt_int8_t   temperature;            // 电机温度 (°C)
    rt_int16_t  current_a;              // A相电流
    rt_int16_t  current_b;              // B相电流 
    rt_int16_t  current_c;              // C相电流
    rt_uint32_t last_update_time;       // 最后更新时间
} ms4010_status3_t;

/* MS4010设备控制块 */
typedef struct
{
    rs485_inst_t       *rs485_handle;   // RS485句柄
    rt_mutex_t          mutex;          // 互斥锁
    rt_uint8_t          motor_id;       // 电机ID
    rt_uint32_t         send_count;     // 发送计数
    rt_uint32_t         error_count;    // 错误计数
    ms4010_status_t     status;         // 电机状态(状态2)
    ms4010_status1_t    status1;        // 详细状态(状态1)
    ms4010_status3_t    status3;        // 相电流状态(状态3)
    rt_uint8_t          tx_buffer[MS4010_MAX_FRAME_SIZE]; // 发送缓冲区
    rt_uint8_t          rx_buffer[MS4010_MAX_FRAME_SIZE]; // 接收缓冲区
} ms4010_device_t;

/* MS4010多电机管理器 */
typedef struct
{
    rs485_inst_t       *rs485_handle;          // 共享的RS485句柄
    rt_mutex_t          bus_mutex;             // 总线互斥锁
    rt_mutex_t          manager_mutex;         // 管理器互斥锁(保护管理器结构体修改)
    ms4010_device_t    *motors[MS4010_MAX_MOTORS]; // 电机设备数组
    rt_uint8_t          motor_count;           // 当前电机数量
    rt_bool_t           initialized;           // 初始化标志
} ms4010_manager_t;

/* 函数声明 */

/* 单电机控制函数（保持向后兼容） */

/**
 * @brief 初始化MS4010电机驱动
 * @param device 设备控制块指针
 * @param rs485_name RS485设备名称
 * @param motor_id 电机ID (1-255)
 * @param baudrate 波特率
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_init(ms4010_device_t *device, const char *rs485_name, rt_uint8_t motor_id, rt_uint32_t baudrate);

/* 多电机管理函数 */

/**
 * @brief 初始化MS4010多电机管理器
 * @param manager 管理器指针
 * @param rs485_name RS485设备名称
 * @param baudrate 波特率
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_manager_init(ms4010_manager_t *manager, const char *rs485_name, rt_uint32_t baudrate);

/**
 * @brief 反初始化MS4010多电机管理器
 * @param manager 管理器指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_manager_deinit(ms4010_manager_t *manager);

/**
 * @brief 添加电机到管理器
 * @param manager 管理器指针
 * @param device 电机设备指针
 * @param motor_id 电机ID (1-255)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_manager_add_motor(ms4010_manager_t *manager, ms4010_device_t *device, rt_uint8_t motor_id);

/**
 * @brief 从管理器移除电机
 * @param manager 管理器指针
 * @param motor_id 电机ID
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_manager_remove_motor(ms4010_manager_t *manager, rt_uint8_t motor_id);

/**
 * @brief 通过ID获取电机设备
 * @param manager 管理器指针
 * @param motor_id 电机ID
 * @return ms4010_device_t* 电机设备指针，NULL表示未找到
 */
ms4010_device_t* ms4010_manager_get_motor(ms4010_manager_t *manager, rt_uint8_t motor_id);

/**
 * @brief 获取所有电机的状态
 * @param manager 管理器指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_manager_get_all_status(ms4010_manager_t *manager);

/**
 * @brief 自动扫描并添加总线上的所有电机
 * @param manager 管理器指针
 * @param start_id 扫描起始ID (默认1)
 * @param end_id 扫描结束ID (默认254)
 * @param timeout_ms 每个ID的扫描超时时间(ms)
 * @return rt_uint8_t 发现的电机数量
 */
rt_uint8_t ms4010_manager_auto_scan(ms4010_manager_t *manager, rt_uint8_t start_id, rt_uint8_t end_id, rt_uint32_t timeout_ms);

/**
 * @brief 快速扫描常用ID范围的电机
 * @param manager 管理器指针
 * @return rt_uint8_t 发现的电机数量
 */
rt_uint8_t ms4010_manager_quick_scan(ms4010_manager_t *manager);

/**
 * @brief 清除所有自动发现的电机
 * @param manager 管理器指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_manager_clear_all(ms4010_manager_t *manager);

/**
 * @brief 反初始化MS4010电机驱动
 * @param device 设备控制块指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_deinit(ms4010_device_t *device);

/**
 * @brief 开环控制
 * @param device 设备控制块指针
 * @param value 控制值 (-1000 ~ 1000)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_open_loop_control(ms4010_device_t *device, rt_int16_t value);

/**
 * @brief 力矩控制
 * @param device 设备控制块指针
 * @param torque 力矩值
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_torque_control(ms4010_device_t *device, rt_int16_t torque);

/**
 * @brief 速度控制
 * @param device 设备控制块指针
 * @param speed 速度值 (dps, 度每秒)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_speed_control(ms4010_device_t *device, rt_int32_t speed);

/**
 * @brief 位置控制(多圈，带速度限制)
 * @param device 设备控制块指针
 * @param angle 目标角度 (0.01度为单位)
 * @param max_speed 最大速度 (0.01dps为单位)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_position_control(ms4010_device_t *device, rt_int32_t angle, rt_uint16_t speed);

/**
 * @brief 位置控制(多圈，带速度限制)
 * @param device 设备控制块指针
 * @param angle 目标角度 (0.01度为单位)
 * @param max_speed 最大速度 (0.01dps为单位)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_position_control2(ms4010_device_t *device, rt_int64_t angle, rt_uint32_t max_speed);

/**
 * @brief 单圈位置控制1
 * @param device 设备控制块指针
 * @param direction 转动方向 (0-顺时针, 1-逆时针)
 * @param angle 目标角度 (0.01度为单位, 0~35999)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_single_position_control1(ms4010_device_t *device, rt_uint8_t direction, rt_uint16_t angle);

/**
 * @brief 单圈位置控制2(带速度限制)
 * @param device 设备控制块指针
 * @param direction 转动方向 (0-顺时针, 1-逆时针)
 * @param angle 目标角度 (0.01度为单位, 0~35999)
 * @param max_speed 最大速度 (0.01dps为单位)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_single_position_control2(ms4010_device_t *device, rt_uint8_t direction, rt_uint16_t angle, rt_uint32_t max_speed);

/**
 * @brief 增量位置控制1
 * @param device 设备控制块指针
 * @param angle_increment 角度增量 (0.01度为单位)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_increment_position_control1(ms4010_device_t *device, rt_int32_t angle_increment);

/**
 * @brief 增量位置控制2(带速度限制)
 * @param device 设备控制块指针
 * @param angle_increment 角度增量 (0.01度为单位)
 * @param max_speed 最大速度 (0.01dps为单位)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_increment_position_control2(ms4010_device_t *device, rt_int32_t angle_increment, rt_uint32_t max_speed);

/**
 * @brief 获取电机状态
 * @param device 设备控制块指针
 * @param status 状态结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_get_status(ms4010_device_t *device, ms4010_status_t *status);

/**
 * @brief 读取电机详细状态1(温度、电压、电流、状态)
 * @param device 设备控制块指针
 * @param status1 状态1结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_read_status1(ms4010_device_t *device, ms4010_status1_t *status1);

/**
 * @brief 清除电机错误标志
 * @param device 设备控制块指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_clear_error(ms4010_device_t *device);

/**
 * @brief 读取电机相电流状态3
 * @param device 设备控制块指针
 * @param status3 状态3结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_read_status3(ms4010_device_t *device, ms4010_status3_t *status3);

/**
 * @brief 电机开启命令
 * @param device 设备控制块指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_motor_on(ms4010_device_t *device);

/**
 * @brief 电机关闭命令
 * @param device 设备控制块指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_motor_off(ms4010_device_t *device);

/**
 * @brief 电机停止命令
 * @param device 设备控制块指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_motor_stop(ms4010_device_t *device);

/**
 * @brief 抱闸控制
 * @param device 设备控制块指针
 * @param brake_cmd 抱闸命令 (MS4010_BRAKE_RELEASE, MS4010_BRAKE_LOCK, MS4010_BRAKE_READ_STATUS)
 * @param brake_status 返回的抱闸状态指针(可为NULL)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_brake_control(ms4010_device_t *device, rt_uint8_t brake_cmd, rt_uint8_t *brake_status);

/**
 * @brief 设置当前位置为任意角度(写入RAM)
 * @param device 设备控制块指针
 * @param angle 设置的角度值 (0.01度为单位)
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_set_current_position(ms4010_device_t *device, rt_int32_t angle);

/**
 * @brief 获取统计信息
 * @param device 设备控制块指针
 * @param send_count 发送计数指针
 * @param error_count 错误计数指针
 * @return rt_err_t 错误码
 */
rt_err_t ms4010_get_statistics(ms4010_device_t *device, rt_uint32_t *send_count, rt_uint32_t *error_count);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_MS4010_H__ */