# MS4010电机驱动测试使用说明

## 概述

本文档描述了如何使用修改后的MS4010电机驱动测试文件。测试文件已经根据新的驱动头文件进行了更新，以确保与驱动接口的兼容性。

## 测试文件

1. **ms4010_test.c** - 完整的测试文件，包含多种测试功能
2. **ms4010_simple_test.c** - 简化的测试文件，用于快速验证
3. **ms4010_sample.c** - 示例文件，展示如何使用MS4010驱动进行各种控制

## 主要修改内容

### 数据结构修改

- **ms4010_status1_t**: 包含温度、电压、电流、电机状态和错误状态
- **ms4010_status_t** (状态2): 包含温度、功率/力矩、速度和编码器值
- **ms4010_status3_t**: 包含温度和三相电流值

### 函数接口修改

- `ms4010_position_control()` → `ms4010_position_control2()`
- 速度参数单位：从 dps 改为 0.01dps (需要乘以100)
- 角度参数单位：0.01度

## 测试命令

### ms4010_test 命令

```bash
# 基本操作
ms4010_test init           # 初始化设备
ms4010_test basic          # 基本通信测试
ms4010_test control        # 控制功能测试
ms4010_test start          # 开始连续测试
ms4010_test stop           # 停止连续测试

# 状态读取
ms4010_test status         # 读取所有状态
ms4010_test stats          # 显示通信统计

# 电机控制
ms4010_test move <speed>   # 速度控制 (dps)
ms4010_test pos <angle>    # 位置控制 (0.01度)
ms4010_test single <dir> <angle>  # 单圈位置控制
ms4010_test increment <angle>     # 增量位置控制
ms4010_test torque <value>        # 力矩控制

# 电机状态控制
ms4010_test motor_on       # 开启电机
ms4010_test motor_off      # 关闭电机
ms4010_test brake <cmd>    # 抱闸控制 (0=锁定, 1=释放, 16=读取)
ms4010_test clear_error    # 清除错误标志
```

### ms4010_simple 命令

```bash
# 基本操作
ms4010_simple start        # 开始简单测试
ms4010_simple init         # 初始化电机
ms4010_simple forward      # 正转测试
ms4010_simple reverse      # 反转测试
ms4010_simple stop         # 停止电机

# 状态读取
ms4010_simple status1      # 读取状态1 (电压、电流等)
ms4010_simple status2      # 读取状态2 (速度、编码器等)
ms4010_simple status3      # 读取状态3 (三相电流)
ms4010_simple stats        # 显示统计信息

# 电机控制
ms4010_simple motor_on     # 开启电机
ms4010_simple motor_off    # 关闭电机
ms4010_simple clear_err    # 清除错误
```

### ms4010_test 命令 (示例文件)

```bash
# 基本操作
ms4010_test status1        # 读取状态1
ms4010_test status2        # 读取状态2
ms4010_test status3        # 读取状态3

# 电机控制
ms4010_test open <value>   # 开环控制
ms4010_test speed <value>  # 速度控制
ms4010_test torque <value> # 力矩控制
ms4010_test pos <angle> [max_speed] # 位置控制
ms4010_test single <dir> <angle>    # 单圈位置控制
ms4010_test increment <angle>       # 增量位置控制

# 电机状态控制
ms4010_test motor_on       # 开启电机
ms4010_test motor_off      # 关闭电机
ms4010_test stop           # 停止电机
ms4010_test brake <cmd>    # 抱闸控制
ms4010_test clear_error    # 清除错误
ms4010_test stats          # 统计信息
```

## 使用示例

### 快速验证
```bash
# 1. 初始化设备
ms4010_simple init

# 2. 开启电机
ms4010_simple motor_on

# 3. 正转测试
ms4010_simple forward

# 4. 查看状态
ms4010_simple status2

# 5. 停止电机
ms4010_simple stop
```

### 完整测试流程
```bash
# 1. 初始化
ms4010_test init

# 2. 基本通信测试
ms4010_test basic

# 3. 控制功能测试
ms4010_test control

# 4. 手动控制测试
ms4010_test move 360        # 360 dps速度
ms4010_test pos 9000        # 90度位置 (90*100)
ms4010_test status          # 查看状态
```

## 注意事项

1. **单位转换**：
   - 角度：输入值为0.01度单位，如90度应输入9000
   - 速度：位置控制中的速度为0.01dps单位，如180dps应输入18000

2. **安全操作**：
   - 测试前确保电机可以安全运行
   - 出现错误时使用 `clear_error` 命令清除错误标志
   - 紧急情况下使用 `motor_off` 关闭电机

3. **通信配置**：
   - 默认使用 uart2 作为RS485设备
   - 默认波特率115200
   - 默认电机ID为1
   - 可在源代码中修改这些配置

## 故障排除

- **通信失败**：检查RS485连接和设备配置
- **控制无效**：确保电机已开启 (`motor_on`)
- **参数错误**：检查输入参数的单位和范围
- **超时错误**：检查电机连接和供电状态

## 开发建议

如需添加新的测试功能，建议：
1. 在简单测试文件中验证基本功能
2. 在完整测试文件中添加详细的测试用例
3. 确保错误处理和状态检查的完整性
