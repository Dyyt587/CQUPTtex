# MS4010电机RS485驱动

## 概述

这是一个基于RT-Thread RS485库开发的MS4010电机驱动程序，支持开环控制、力矩控制、速度控制和位置控制等多种控制模式。

## 特性

- ✅ 基于RT-Thread RS485库，支持多种传输模式
- ✅ 支持多种控制模式：开环、力矩、速度、位置控制
- ✅ 完整的错误处理和校验机制
- ✅ 线程安全的设计
- ✅ 统计信息支持
- ✅ MSH命令行支持
- ✅ 详细的调试日志

## 文件结构

```
applications/
├── drv_ms4010.h        # 驱动头文件
├── drv_ms4010.c        # 驱动实现
├── ms4010_sample.c     # 使用示例
├── ms4010_config.h     # 配置文件
└── README_MS4010.md    # 说明文档
```

## 快速开始

### 1. 配置RS485设备

确保你的项目中已经正确配置了RS485设备，比如：

```c
// 在 packages/rs485-latest/src/rs485_dev_cfg.h 中配置
#define RS485_DEV_CFG_TABLE \
{ \
    {"uart2", 115200, 0, -1, 0, "rs485_0"}, \
}
```

### 2. 初始化MS4010电机

```c
#include "drv_ms4010.h"

ms4010_device_t motor_device;

// 初始化电机（使用uart2，电机ID为1，波特率115200）
rt_err_t result = ms4010_init(&motor_device, "uart2", 1, 115200);
if (result != RT_EOK)
{
    rt_kprintf("Motor initialization failed\n");
    return;
}
```

### 3. 控制电机

```c
// 开环控制 (-1000 ~ 1000)
ms4010_open_loop_control(&motor_device, 500);

// 速度控制 (单位：度/秒)
ms4010_speed_control(&motor_device, 360); // 360度/秒

// 位置控制 (角度单位：0.01度，速度单位：度/秒)
ms4010_position_control(&motor_device, 18000, 360); // 转到180度位置

// 力矩控制
ms4010_torque_control(&motor_device, 1000);
```

### 4. 获取电机状态

```c
ms4010_status_t status;
rt_err_t result = ms4010_get_status(&motor_device, &status);
if (result == RT_EOK)
{
    rt_kprintf("Temperature: %d°C\n", status.temperature);
    rt_kprintf("Speed: %d rpm\n", status.speed);
    rt_kprintf("Encoder: %d\n", status.encoder);
}
```

## API参考

### 初始化/反初始化

#### `ms4010_init()`
```c
rt_err_t ms4010_init(ms4010_device_t *device, const char *rs485_name, rt_uint8_t motor_id, rt_uint32_t baudrate);
```
初始化MS4010电机驱动。

**参数：**
- `device`：设备控制块指针
- `rs485_name`：RS485设备名称（如"uart2"）
- `motor_id`：电机ID（1-255）
- `baudrate`：通信波特率

**返回值：**
- `RT_EOK`：成功
- 负值：错误码

#### `ms4010_deinit()`
```c
rt_err_t ms4010_deinit(ms4010_device_t *device);
```
反初始化MS4010电机驱动。

### 控制函数

#### `ms4010_open_loop_control()`
```c
rt_err_t ms4010_open_loop_control(ms4010_device_t *device, rt_int16_t value);
```
开环控制电机。

**参数：**
- `value`：控制值（-1000 ~ 1000）

#### `ms4010_speed_control()`
```c
rt_err_t ms4010_speed_control(ms4010_device_t *device, rt_int32_t speed);
```
速度控制电机。

**参数：**
- `speed`：目标速度（单位：度/秒）

#### `ms4010_position_control()`
```c
rt_err_t ms4010_position_control(ms4010_device_t *device, rt_int32_t angle, rt_uint16_t speed);
```
位置控制电机。

**参数：**
- `angle`：目标角度（单位：0.01度）
- `speed`：最大速度（单位：度/秒）

#### `ms4010_torque_control()`
```c
rt_err_t ms4010_torque_control(ms4010_device_t *device, rt_int16_t torque);
```
力矩控制电机。

**参数：**
- `torque`：目标力矩值

### 状态查询

#### `ms4010_get_status()`
```c
rt_err_t ms4010_get_status(ms4010_device_t *device, ms4010_status_t *status);
```
获取电机状态信息。

#### `ms4010_get_statistics()`
```c
rt_err_t ms4010_get_statistics(ms4010_device_t *device, rt_uint32_t *send_count, rt_uint32_t *error_count);
```
获取统计信息。

## MSH命令

如果启用了MSH支持，可以使用以下命令：

```bash
# 查看电机状态
ms4010_test status

# 开环控制
ms4010_test open 500

# 速度控制
ms4010_test speed 360

# 位置控制
ms4010_test pos 18000

# 停止电机
ms4010_test stop
```

## 配置选项

在`ms4010_config.h`中可以配置以下选项：

```c
#define MS4010_RS485_DEVICE_NAME            "uart2"     // RS485设备名称
#define MS4010_DEFAULT_BAUDRATE             115200      // 默认波特率
#define MS4010_DEFAULT_MOTOR_ID             1           // 默认电机ID
#define MS4010_COMMUNICATION_TIMEOUT        1000        // 通信超时时间(ms)
#define MS4010_DEBUG_ENABLE                 1           // 启用调试输出
#define MS4010_SAMPLE_ENABLE                1           // 启用示例代码
#define MS4010_MSH_CMD_ENABLE               1           // 启用MSH命令
```

## 错误码

| 错误码 | 值 | 说明 |
|--------|-----|------|
| MS4010_EOK | 0 | 成功 |
| MS4010_ERROR | -1 | 通用错误 |
| MS4010_ETIMEOUT | -2 | 通信超时 |
| MS4010_EINVAL | -3 | 参数错误 |
| MS4010_EFRAME | -4 | 帧格式错误 |
| MS4010_ECHECKSUM | -5 | 校验和错误 |

## 协议格式

MS4010使用以下帧格式：

**发送帧：**
```
| 帧头 | 命令 | 电机ID | 数据长度 | 头校验 | 数据 | 数据校验 |
| 0x3E |  1B  |   1B   |    1B    |   1B   | 0-8B |    1B    |
```

**响应帧：**
```
| 帧头 | 命令 | 电机ID | 数据长度 | 头校验 | 温度 | 功率/力矩 | 速度 | 编码器 | 数据校验 |
| 0x3E |  1B  |   1B   |   0x07   |   1B   |  1B  |    2B     |  2B  |   2B   |    1B    |
```

## 注意事项

1. 确保RS485硬件连接正确
2. 电机ID必须与实际硬件设置一致
3. 波特率必须与电机设置匹配
4. 在多线程环境中使用时，驱动已经提供了线程安全保护
5. 建议在初始化后先测试通信是否正常

## 故障排除

### 1. 初始化失败
- 检查RS485设备名称是否正确
- 确认RS485设备已正确配置并注册到系统

### 2. 通信超时
- 检查硬件连接
- 确认波特率设置
- 检查电机ID是否正确

### 3. 校验和错误
- 可能是通信干扰或硬件问题
- 检查RS485总线终端电阻

### 4. 帧格式错误
- 确认电机固件版本与协议匹配
- 检查是否有其他设备在总线上发送数据

## 更新日志

- v1.0 (2025-07-25)
  - 初始版本
  - 支持基本的电机控制功能
  - 基于RT-Thread RS485库实现
