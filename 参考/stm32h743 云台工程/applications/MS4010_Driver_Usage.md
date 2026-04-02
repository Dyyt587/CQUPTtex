# MS4010电机驱动使用说明

## 概述
MS4010电机驱动是基于RT-Thread操作系统的485通信电机驱动，支持完整的MS4010伺服电机控制协议。

## 文件结构
```
applications/
├── drv_ms4010.h          # 驱动头文件
├── drv_ms4010.c          # 驱动实现文件
├── ms4010_sample.c       # 使用示例
├── ms4010_simple_test.c  # 简单测试
├── ms4010_test.c         # 高级测试
└── MS4010_Driver_Usage.md # 本说明文档
```

## 主要功能

### 1. 支持的控制模式
- **开环控制**: 直接电压控制
- **转矩控制**: 电流环控制 (单位: 0.01A)
- **速度控制**: 速度环控制 (单位: dps, 度/秒)
- **位置控制**: 多种位置控制模式
  - 基础位置控制 (角度 + 最大速度)
  - 单圈位置控制 (方向控制)
  - 增量位置控制 (相对移动)

### 2. 状态读取
- **状态1**: 温度、电流、速度、编码器
- **状态2**: 基础状态信息
- **状态3**: 完整状态 (温度、电压、错误、角度、速度、编码器)

### 3. 电机控制
- 电机开启/关闭
- 电机停止
- 抱闸控制
- 错误清除

## API 函数说明

### 初始化函数
```c
rt_err_t ms4010_init(ms4010_device_t *device, const char *rs485_name, 
                     rt_uint8_t motor_id, rt_uint32_t baudrate);
```

### 控制函数
```c
// 开环控制 (功率值: -1000 ~ 1000)
rt_err_t ms4010_open_loop_control(ms4010_device_t *device, rt_int16_t value);

// 转矩控制 (电流值: 单位0.01A)
rt_err_t ms4010_torque_control(ms4010_device_t *device, rt_int16_t torque);

// 速度控制 (速度: 单位dps)
rt_err_t ms4010_speed_control(ms4010_device_t *device, rt_int32_t speed);

// 位置控制 (角度: 单位0.01度, 最大速度: 单位dps)
rt_err_t ms4010_position_control(ms4010_device_t *device, rt_int32_t angle, rt_uint16_t max_speed);
```

### 状态读取函数
```c
// 读取状态1 (温度、电流、速度、编码器)
rt_err_t ms4010_read_status1(ms4010_device_t *device, ms4010_status1_t *status1);

// 读取状态3 (完整状态信息)
rt_err_t ms4010_read_status3(ms4010_device_t *device, ms4010_status3_t *status3);
```

### 电机控制函数
```c
// 电机开启/关闭/停止
rt_err_t ms4010_motor_on(ms4010_device_t *device);
rt_err_t ms4010_motor_off(ms4010_device_t *device);
rt_err_t ms4010_motor_stop(ms4010_device_t *device);

// 抱闸控制
rt_err_t ms4010_brake_control(ms4010_device_t *device, rt_uint8_t brake_cmd, rt_uint8_t *brake_status);

// 清除错误
rt_err_t ms4010_clear_error(ms4010_device_t *device);
```

### 统计信息
```c
rt_err_t ms4010_get_statistics(ms4010_device_t *device, rt_uint32_t *send_count, rt_uint32_t *error_count);
```

## 使用示例

### 1. 基本初始化
```c
#include "drv_ms4010.h"

static ms4010_device_t motor;

int main(void)
{
    // 初始化电机 (使用uart2, 电机ID=1, 波特率115200)
    rt_err_t result = ms4010_init(&motor, "uart2", 1, 115200);
    if (result != RT_EOK)
    {
        rt_kprintf("Motor init failed: %d\n", result);
        return -1;
    }
    
    return 0;
}
```

### 2. 速度控制示例
```c
// 设置速度为360度/秒
ms4010_speed_control(&motor, 360);

// 等待2秒
rt_thread_mdelay(2000);

// 停止电机
ms4010_motor_stop(&motor);
```

### 3. 位置控制示例
```c
// 移动到90度位置，最大速度180dps
ms4010_position_control(&motor, 9000, 180);  // 90度 = 90 * 100

// 等待到达位置
rt_thread_mdelay(3000);

// 读取当前状态
ms4010_status1_t status;
if (ms4010_read_status1(&motor, &status) == RT_EOK)
{
    rt_kprintf("Current position: %.2f degrees\n", status.encoder * 0.01f);
}
```

## MSH命令测试

### 1. 简单测试命令
```bash
# 初始化电机
ms4010_simple init

# 正转测试
ms4010_simple forward

# 反转测试  
ms4010_simple reverse

# 停止电机
ms4010_simple stop

# 读取状态
ms4010_simple status1
ms4010_simple status3

# 显示统计
ms4010_simple stats
```

### 2. 详细测试命令
```bash
# 初始化
ms4010_test init

# 基础通信测试
ms4010_test basic

# 控制功能测试
ms4010_test control

# 开启连续测试
ms4010_test start

# 停止连续测试
ms4010_test stop

# 查看状态
ms4010_test status

# 统计信息
ms4010_test stats
```

### 3. 完整功能测试
```bash
# 状态读取
ms4010_test status1     # 读取状态1
ms4010_test status3     # 读取状态3

# 开环控制
ms4010_test open 200    # 功率200

# 速度控制
ms4010_test speed 360   # 360dps

# 转矩控制
ms4010_test torque 100  # 1.00A电流

# 位置控制
ms4010_test pos 9000 180  # 90度，最大速度180dps

# 电机控制
ms4010_test motor_on    # 开启电机
ms4010_test motor_off   # 关闭电机
ms4010_test stop        # 停止电机

# 抱闸控制
ms4010_test brake 1     # 抱闸锁定
ms4010_test brake 0     # 抱闸释放

# 错误清除
ms4010_test clear_error

# 统计信息
ms4010_test stats
```

## 错误码说明
- `RT_EOK (0)`: 成功
- `RT_ERROR (-1)`: 一般错误
- `RT_ETIMEOUT (-2)`: 超时错误
- `RT_EINVAL (-22)`: 参数错误

## 注意事项

1. **硬件连接**: 确保RS485硬件连接正确，包括A+、B-线序
2. **波特率匹配**: 驱动波特率必须与电机设置的波特率一致
3. **电机ID**: 每个电机的ID必须唯一
4. **线程安全**: 驱动内部使用互斥锁保证线程安全
5. **错误处理**: 建议检查每个函数的返回值进行错误处理
6. **单位注意**: 
   - 角度单位: 0.01度 (9000 = 90度)
   - 速度单位: dps (度/秒)
   - 电流单位: 0.01A (100 = 1.00A)
   - 电压单位: 0.1V

## 调试建议

1. 使用`ms4010_simple`命令进行基础功能验证
2. 查看统计信息判断通信质量
3. 使用状态读取功能监控电机运行状态
4. 遇到问题时先使用`clear_error`清除错误标志

## 更新历史
- v1.0: 初始版本，支持完整的MS4010协议
- 基于RT-Thread RS485库实现
- 支持多种控制模式和状态读取
- 提供完整的测试用例和MSH命令
