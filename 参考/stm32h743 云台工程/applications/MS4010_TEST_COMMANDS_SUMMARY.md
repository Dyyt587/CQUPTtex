# MS4010驱动测试命令完整总结

## 概述
MS4010电机驱动已完善测试文件，支持所有可用函数的MSH命令测试。共有三个测试文件，每个文件都提供了完整的功能测试。

## 测试文件说明

### 1. ms4010_test.c - 主要综合测试
- **MSH命令**: `ms4010_test`
- **功能**: 提供最全面的测试功能，包括自动化测试和单独功能测试
- **线程支持**: 支持连续测试线程

### 2. ms4010_simple_test.c - 简单快速测试  
- **MSH命令**: `ms4010_simple` 
- **功能**: 提供快速验证和简单测试功能
- **线程支持**: 支持自动化测试线程

### 3. ms4010_sample.c - 示例演示
- **MSH命令**: `ms4010_sample`
- **功能**: 提供使用示例和参考实现
- **线程支持**: 支持控制演示线程

## 支持的驱动函数完整列表

| 函数名 | ms4010_test | ms4010_simple | ms4010_sample | 说明 |
|--------|-------------|---------------|---------------|------|
| `ms4010_init` | ✅ init | ✅ init | ✅ init | 初始化电机驱动 |
| `ms4010_deinit` | ✅ deinit | ✅ deinit | ✅ deinit | 反初始化电机驱动 |
| `ms4010_open_loop_control` | ✅ open | ✅ open | ✅ open | 开环控制(-1000~1000) |
| `ms4010_torque_control` | ✅ torque | ✅ torque | ✅ torque | 力矩控制 |
| `ms4010_speed_control` | ✅ move | ✅ speed | ✅ speed | 速度控制(dps) |
| `ms4010_position_control` | ✅ pos | ✅ pos | ✅ pos | 位置控制(原版) |
| `ms4010_position_control2` | ✅ pos2 | ✅ pos2 | ✅ pos2 | 位置控制2(多圈+速度限制) |
| `ms4010_single_position_control1` | ✅ single1 | ✅ single1 | ✅ single1 | 单圈位置控制1 |
| `ms4010_single_position_control2` | ✅ single2 | ✅ single2 | ✅ single2 | 单圈位置控制2(带速度) |
| `ms4010_increment_position_control1` | ✅ increment1 | ✅ inc1 | ✅ inc1 | 增量位置控制1 |
| `ms4010_increment_position_control2` | ✅ increment2 | ✅ inc2 | ✅ inc2 | 增量位置控制2(带速度) |
| `ms4010_get_status` | ✅ status | ✅ status2 | ✅ status2 | 获取电机状态2 |
| `ms4010_read_status1` | ✅ status | ✅ status1 | ✅ status1 | 读取电机状态1 |
| `ms4010_read_status3` | ✅ status | ✅ status3 | ✅ status3 | 读取电机状态3(相电流) |
| `ms4010_motor_on` | ✅ motor_on | ✅ motor_on | ✅ motor_on | 电机开启 |
| `ms4010_motor_off` | ✅ motor_off | ✅ motor_off | ✅ motor_off | 电机关闭 |
| `ms4010_motor_stop` | ✅ motor_stop | ✅ motor_stop | ✅ motor_stop | 电机立即停止 |
| `ms4010_brake_control` | ✅ brake | ✅ brake | ✅ brake | 抱闸控制 |
| `ms4010_clear_error` | ✅ clear_error | ✅ clear_err | ✅ clear_error | 清除错误标志 |
| `ms4010_set_current_position` | ✅ set_pos | ✅ set_pos | ✅ set_pos | 设置当前位置为任意角度 |
| `ms4010_get_statistics` | ✅ stats | ✅ stats | ✅ stats | 获取统计信息 |

## 命令使用说明

### 基本命令格式
```bash
# 主测试文件
ms4010_test <command> [parameters]

# 简单测试文件  
ms4010_simple <command> [parameters]

# 示例文件
ms4010_sample <command> [parameters]
```

### 常用命令示例

#### 初始化和基本测试
```bash
# 初始化电机
ms4010_test init

# 查看帮助
ms4010_test

# 运行基本通信测试
ms4010_test basic

# 运行控制功能测试
ms4010_test control
```

#### 控制命令
```bash
# 开环控制
ms4010_test open 300

# 速度控制
ms4010_test move 1800    # 18度/秒

# 位置控制
ms4010_test pos 36000 18000    # 360度，速度180度/秒
ms4010_test pos2 36000 180000  # 360度，最大速度1800度/秒

# 单圈位置控制
ms4010_test single1 0 18000    # 顺时针转180度
ms4010_test single2 1 18000 36000  # 逆时针转180度，最大速度360度/秒

# 增量位置控制
ms4010_test increment1 9000   # 增量90度
ms4010_test increment2 9000 36000  # 增量90度，最大速度360度/秒

# 力矩控制
ms4010_test torque 500
```

#### 状态查询
```bash
# 查看所有状态
ms4010_test status

# 查看统计信息
ms4010_test stats
```

#### 电机控制
```bash
# 电机开关
ms4010_test motor_on
ms4010_test motor_off
ms4010_test motor_stop

# 抱闸控制
ms4010_test brake 1    # 释放抱闸
ms4010_test brake 0    # 锁定抱闸
ms4010_test brake 16   # 读取抱闸状态

# 清除错误
ms4010_test clear_error

# 设置当前位置
ms4010_test set_pos 36000    # 设置当前位置为360度
```

#### 连续测试
```bash
# 启动连续测试
ms4010_test start

# 停止连续测试
ms4010_test stop
```

## 参数说明

### 角度参数
- 所有角度参数以0.01度为单位
- 例如：36000 = 360.00度

### 速度参数
- `ms4010_position_control`: 速度单位为度/秒
- `ms4010_position_control2`: 最大速度单位为0.01度/秒
- `ms4010_speed_control`: 速度单位为度/秒

### 方向参数
- 0: 顺时针
- 1: 逆时针

### 抱闸命令
- 0: 锁定抱闸
- 1: 释放抱闸
- 16: 读取抱闸状态

## 错误处理
所有命令都会返回执行结果：
- SUCCESS: 命令执行成功
- FAILED: 命令执行失败

可通过以下方式查看详细错误信息：
- 查看日志输出
- 使用`stats`命令查看通信统计
- 使用`status`命令查看电机状态和错误标志

## 注意事项
1. 使用前需要先执行`init`命令初始化电机
2. 确保RS485通信正常配置
3. 注意电机参数范围，避免超出限制
4. 测试时注意安全，确保电机周围无障碍物
5. 可以同时使用多个测试文件的命令进行不同类型的测试

## 配置参数
默认配置在各测试文件中定义：
- RS485设备名: "uart2"
- 电机ID: 1
- 波特率: 115200
- 超时时间: 1000ms

如需修改配置，请编辑对应测试文件中的宏定义。
