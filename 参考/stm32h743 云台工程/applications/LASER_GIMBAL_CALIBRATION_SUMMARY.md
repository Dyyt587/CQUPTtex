# 激光云台角度偏移校准功能总结

## 🎯 新增功能概述

我已经为你的激光云台系统添加了完整的角度偏移校准功能，解决了电机安装角度不一致的问题。现在你可以通过手动调整云台位置来获取和设置正确的安装偏移角度。

## 📋 新增的功能特性

### 1. 校准模式
- ✅ **校准模式开关**: 启用/禁用直接电机控制
- ✅ **绕过角度限制**: 校准时不受角度限位约束
- ✅ **直接电机控制**: 不经过偏移计算，直接控制电机原始角度

### 2. 手动调整功能
- ✅ **绝对角度设置**: 直接设置电机到指定原始角度
- ✅ **增量角度调整**: 基于当前位置进行微调
- ✅ **实时角度显示**: 查看当前电机原始角度和偏移值

### 3. 自动偏移计算
- ✅ **偏移量自动计算**: 根据当前位置和期望角度自动计算偏移
- ✅ **支持任意参考点**: 可设置任意角度作为参考位置
- ✅ **考虑电机反向**: 自动处理电机安装方向问题

### 4. 参数持久化
- ✅ **保存到文件**: 将校准参数保存到配置文件
- ✅ **从文件加载**: 系统启动时自动加载校准参数
- ✅ **多文件支持**: 支持保存多组校准参数

## 🔧 新增的API接口

### 核心校准API
```c
// 进入/退出校准模式
rt_err_t laser_gimbal_calibration_mode(laser_gimbal_t *gimbal, bool enable);

// 校准模式下直接设置电机角度
rt_err_t laser_gimbal_calibration_set_motor_angle(laser_gimbal_t *gimbal,
                                                  float yaw_motor_angle,
                                                  float pitch_motor_angle);

// 校准模式下微调电机角度
rt_err_t laser_gimbal_calibration_adjust_angle(laser_gimbal_t *gimbal,
                                               float yaw_delta,
                                               float pitch_delta);

// 获取当前电机原始角度
rt_err_t laser_gimbal_get_motor_raw_angle(laser_gimbal_t *gimbal,
                                         float *yaw_motor_angle,
                                         float *pitch_motor_angle);

// 计算并设置角度偏移
rt_err_t laser_gimbal_calibrate_offset(laser_gimbal_t *gimbal,
                                      float target_yaw,
                                      float target_pitch);

// 保存/加载校准参数
rt_err_t laser_gimbal_save_calibration(laser_gimbal_t *gimbal, const char *filename);
rt_err_t laser_gimbal_load_calibration(laser_gimbal_t *gimbal, const char *filename);
```

## 📱 新增的命令行接口

| 命令 | 功能 | 示例 |
|------|------|------|
| `laser_calibration on/off` | 进入/退出校准模式 | `laser_calibration on` |
| `laser_cal_set <yaw> <pitch>` | 设置电机绝对角度 | `laser_cal_set 0 0` |
| `laser_cal_adjust <±yaw> <±pitch>` | 增量调整角度 | `laser_cal_adjust 1.5 -0.5` |
| `laser_cal_show` | 显示当前角度和偏移 | `laser_cal_show` |
| `laser_cal_offset <yaw> <pitch>` | 设置角度参考点 | `laser_cal_offset 0 0` |
| `laser_cal_save [file]` | 保存校准参数 | `laser_cal_save` |
| `laser_cal_load [file]` | 加载校准参数 | `laser_cal_load` |
| `laser_cal_wizard` | 显示校准向导 | `laser_cal_wizard` |

## 🎮 典型校准流程

### 快速校准（推荐）
```shell
# 1. 进入校准模式
msh> laser_calibration on

# 2. 手动调整云台指向正前方
msh> laser_cal_set 0.0 0.0          # 从零位开始
msh> laser_cal_adjust 3.2 -1.5      # 大幅调整
msh> laser_cal_adjust 0.3 0.1       # 精细调整
msh> laser_cal_adjust -0.05 0.0     # 微调

# 3. 当激光完全指向正前方时，设置偏移
msh> laser_cal_offset 0.0 0.0

# 4. 保存校准参数
msh> laser_cal_save

# 5. 退出校准模式
msh> laser_calibration off

# 6. 验证校准结果
msh> laser_angle 0.0 0.0    # 应指向正前方
msh> laser_angle 45.0 0.0   # 应指向右侧45°
```

### 程序化校准
```c
// 在系统初始化时自动加载校准参数
laser_gimbal_load_calibration(&gimbal, NULL);

// 程序中进行校准
laser_gimbal_calibration_mode(&gimbal, true);
laser_gimbal_calibration_set_motor_angle(&gimbal, 0.0f, 0.0f);
// ... 手动调整或程序化调整
laser_gimbal_calibrate_offset(&gimbal, 0.0f, 0.0f);
laser_gimbal_save_calibration(&gimbal, NULL);
laser_gimbal_calibration_mode(&gimbal, false);
```

## 🔍 工作原理

### 角度转换关系
```
正常模式：
实际电机角度 = 逻辑角度 + 偏移量 + (反向处理)

校准模式：
实际电机角度 = 直接输入角度 (绕过偏移和反向)

偏移计算：
偏移量 = 期望逻辑角度 - 当前电机原始角度
```

### 数据流程
```
用户输入逻辑角度 (如: 30°, 15°)
         ↓
    应用机械参数
    (偏移 + 反向)
         ↓
    转换为电机单位
    (×100, MS4010使用0.01°)
         ↓
     发送给电机
```

## 📁 文件结构更新

### 新增/修改的文件
- `laser_gimbal.h` - 添加了校准相关的结构体成员和函数声明
- `laser_gimbal.c` - 实现了完整的校准功能
- `laser_gimbal_test.c` - 添加了校准相关的MSH命令
- `laser_gimbal_example.c` - 添加了校准功能演示
- `LASER_GIMBAL_CALIBRATION_GUIDE.md` - 详细的校准使用指南

### 核心数据结构更新
```c
typedef struct {
    // ... 原有成员
    
    /* 校准相关参数 */
    bool calibration_mode;              // 校准模式标志
    float yaw_motor_current_angle;      // 偏航轴电机当前原始角度
    float pitch_motor_current_angle;    // 俯仰轴电机当前原始角度
    
    // ... 其他成员
} laser_gimbal_t;
```

## 🎯 使用场景

### 1. 初次安装校准
- 电机安装完成后的首次校准
- 确保0°角度对应实际的正前方

### 2. 维护校准
- 定期重新校准以补偿机械松动
- 环境温度变化后的精度调整

### 3. 多环境校准
- 室内外不同环境的校准参数
- 不同应用场景的专用校准

### 4. 故障诊断
- 通过校准模式检测机械问题
- 验证电机通信和控制是否正常

## 💡 最佳实践

### 校准技巧
1. **准备工作充分**: 使用激光指示器和适当距离的目标
2. **分步调整**: 先大幅调整，再精细微调
3. **多点验证**: 不仅校准0°，还要验证其他角度
4. **定期维护**: 建议每月重新校准一次

### 安全注意事项
1. **校准模式下无限位保护**: 注意不要超出机械极限
2. **逐步调整**: 避免大幅度跳跃式调整
3. **备份参数**: 校准前备份原有参数文件

## 🚀 未来扩展

### 可能的增强功能
1. **自动校准**: 结合视觉系统实现自动校准
2. **多点校准**: 支持多个参考点的复合校准
3. **温度补偿**: 基于温度的动态偏移调整
4. **磨损补偿**: 长期使用后的自适应校准

## ✅ 测试验证

### 基本功能测试
```shell
# 测试校准功能演示
msh> calibration_demo_example

# 查看校准向导
msh> laser_cal_wizard

# 验证角度精度
msh> laser_angle 0 0      # 检查是否指向正前方
msh> laser_angle 90 0     # 检查是否指向正右方
```

### 精度验证方法
1. 使用量角器测量实际角度
2. 多个距离测试坐标精度
3. 长时间稳定性测试

---

通过这套完整的校准系统，你的激光云台现在可以轻松应对电机安装角度不一致的问题，实现高精度的角度控制和坐标指向功能。校准过程简单直观，支持手动微调和自动计算，确保系统在各种安装条件下都能正常工作。
