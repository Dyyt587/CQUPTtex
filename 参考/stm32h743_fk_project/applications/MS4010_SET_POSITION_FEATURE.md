# MS4010设置当前位置功能新增

## 功能说明

新增了`ms4010_set_current_position`函数，用于设置电机的当前位置为任意角度（写入RAM）。这个功能允许用户重新定义电机的零点位置，而不需要物理移动电机。

## 协议详情

### 命令码
- **命令码**: 0x95
- **数据长度**: 4字节
- **响应长度**: 标准控制响应(13字节)

### 数据格式
多圈角度值为int32_t类型数据，单位为0.01°/LSB：
- DATA[0]: 角度低字节1 
- DATA[1]: 角度字节2
- DATA[2]: 角度字节3  
- DATA[3]: 角度高字节4

### 取值范围
- 数据类型: int32_t
- 单位: 0.01度
- 范围: -2,147,483,648 ~ 2,147,483,647 (约-21,474,836.48° ~ 21,474,836.47°)

## 驱动接口

### 函数原型
```c
rt_err_t ms4010_set_current_position(ms4010_device_t *device, rt_int32_t angle);
```

### 参数说明
- `device`: MS4010设备控制块指针
- `angle`: 要设置的角度值，单位为0.01度

### 返回值
- `RT_EOK`: 设置成功
- 其他值: 设置失败

## 测试命令支持

### ms4010_test
```bash
ms4010_test set_pos <angle>
```

### ms4010_simple
```bash
ms4010_simple set_pos <angle>
```

### ms4010_sample
```bash
ms4010_sample set_pos <angle>
```

## 使用示例

### 基本使用
```bash
# 设置当前位置为0度（重置零点）
ms4010_test set_pos 0

# 设置当前位置为90度
ms4010_test set_pos 9000

# 设置当前位置为-180度
ms4010_test set_pos -18000

# 设置当前位置为720度（两圈）
ms4010_test set_pos 72000
```

### 典型应用场景

#### 1. 零点校准
```bash
# 手动将电机移动到期望的零点位置
ms4010_test pos 0  # 先移动到当前的零点
# 手动调整电机到实际的零点位置
# 然后设置当前位置为零点
ms4010_test set_pos 0
```

#### 2. 位置偏移补偿
```bash
# 如果发现位置有固定偏移，可以通过设置当前位置来补偿
# 例如，实际位置比读取位置多10度
ms4010_test set_pos 1000  # 补偿10度偏移
```

#### 3. 多圈计数重置
```bash
# 重置多圈计数器
ms4010_test set_pos 0
```

## 注意事项

1. **仅写入RAM**: 此设置仅保存到RAM中，断电后会丢失
2. **不影响物理位置**: 此命令不会移动电机，只是重新定义当前位置的角度值
3. **多圈支持**: 支持多圈角度设置，可以设置超过360度的值
4. **立即生效**: 设置后立即生效，后续的位置读取和控制都基于新的零点
5. **精度**: 角度精度为0.01度

## 与其他命令的配合使用

### 位置校准流程
```bash
# 1. 初始化电机
ms4010_test init

# 2. 开启电机
ms4010_test motor_on

# 3. 移动到期望的零点位置（如果需要）
ms4010_test pos 0

# 4. 设置当前位置为零点
ms4010_test set_pos 0

# 5. 验证设置
ms4010_test status

# 6. 测试位置控制
ms4010_test pos 9000  # 移动到90度
```

### 误差补偿流程
```bash
# 1. 读取当前状态
ms4010_test status

# 2. 如果发现位置偏差（比如实际0度位置显示为500，即5度）
# 3. 设置当前位置来补偿偏差
ms4010_test set_pos 0  # 重新定义当前位置为0度

# 4. 验证补偿效果
ms4010_test status
```

## 实现细节

### 驱动层实现
```c
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
```

这个新功能为MS4010电机驱动提供了灵活的位置校准能力，特别适用于需要精确位置控制的应用场景。
