/**
 * @file ms4010_config.h
 * @brief MS4010电机驱动配置文件
 * @version 1.0
 * @date 2025-07-25
 * @author Your Name
 * @copyright Copyright (c) 2025
 */

#ifndef __MS4010_CONFIG_H__
#define __MS4010_CONFIG_H__

/* MS4010配置选项 */

/* 使用的RS485设备名称 */
#ifndef MS4010_RS485_DEVICE_NAME
#define MS4010_RS485_DEVICE_NAME            "uart2"
#endif

/* 默认波特率 */
#ifndef MS4010_DEFAULT_BAUDRATE
#define MS4010_DEFAULT_BAUDRATE             115200
#endif

/* 默认电机ID */
#ifndef MS4010_DEFAULT_MOTOR_ID
#define MS4010_DEFAULT_MOTOR_ID             1
#endif

/* 通信超时时间(ms) */
#ifndef MS4010_COMMUNICATION_TIMEOUT
#define MS4010_COMMUNICATION_TIMEOUT        1000
#endif

/* 使能调试输出 */
#ifndef MS4010_DEBUG_ENABLE
#define MS4010_DEBUG_ENABLE                 1
#endif

/* 使能示例代码 */
#ifndef MS4010_SAMPLE_ENABLE
#define MS4010_SAMPLE_ENABLE                1
#endif

/* 使能MSH命令支持 */
#ifndef MS4010_MSH_CMD_ENABLE
#define MS4010_MSH_CMD_ENABLE               1
#endif

#endif /* __MS4010_CONFIG_H__ */
