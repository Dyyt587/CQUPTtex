# 激光云台自动校准功能使用指南

## 🎯 功能特性

新的自动校准功能让校准过程变得极其简单：
- **电机自动关闭** - 进入校准模式时电机关闭，允许手动调整
- **实时角度监控** - 持续显示当前角度信息
- **智能稳定检测** - 自动检测角度何时稳定
- **自动保存校准** - 角度稳定后自动计算并保存偏移量

## 🔧 工作原理

### 传统校准 vs 自动校准

**传统方式（旧）：**
```
命令控制 → 微调角度 → 手动保存
```

**自动校准（新）：**
```
电机关闭 → 手动调整 → 自动检测稳定 → 自动保存
```

### 技术实现
1. **电机控制**：进入校准模式时自动关闭扭矩，退出时重新启用
2. **监控线程**：后台线程持续监控角度变化
3. **稳定检测**：角度变化小于阈值且持续一定时间即认为稳定
4. **自动保存**：检测到稳定后自动计算偏移量并保存到文件

## 🚀 快速使用指南

### 基本校准流程（推荐）

```shell
# 1. 进入校准模式
msh> laser_calibration on
# → 电机关闭，监控开始

# 2. 手动调整云台
#    用手调整云台，使激光指向目标中心（正前方0°位置）

# 3. 保持稳定
#    保持位置不动，等待控制台显示稳定状态
#    默认3秒后自动保存："✅ Auto-calibration completed!"

# 4. 退出校准模式
msh> laser_calibration off
# → 电机重新启用

# 5. 测试校准结果
msh> laser_angle 0 0      # 应指向目标中心
msh> laser_angle 30 0     # 应指向右侧30°
```

### 控制台输出示例

```
msh> laser_calibration on
Calibration mode ENABLED:
  - Motors are OFF (manual adjustment allowed)
  - Real-time angle monitoring started
  - Auto-save when angles stable for 3.0s
Use your hands to adjust gimbal to desired position

📐 Current angles: Yaw=5.23°, Pitch=-1.45° (Stable: 0.8s/3.0s)
📐 Current angles: Yaw=5.24°, Pitch=-1.44° (Stable: 1.2s/3.0s)
📐 Current angles: Yaw=5.23°, Pitch=-1.45° (Stable: 2.1s/3.0s)
📐 Current angles: Yaw=5.23°, Pitch=-1.45° (✅ STABLE - Will auto-save)

Angles stable for 3.0s - Auto-saving calibration...
✅ Auto-calibration completed!
   Final angles: Yaw=5.23°, Pitch=-1.45°
   Calculated offsets: Yaw=5.23°, Pitch=-1.45°
```

## ⚙️ 高级配置

### 监控参数配置

```shell
# 查看当前设置
msh> laser_cal_config

# 自定义配置：0.3°阈值，2秒稳定时间，启用自动保存
msh> laser_cal_config 0.3 2.0 1

# 禁用自动保存（需要手动保存）
msh> laser_cal_config 0.5 3.0 0
```

### 参数说明

| 参数 | 说明 | 推荐值 | 范围 |
|------|------|---------|------|
| **稳定阈值** | 角度变化小于此值认为稳定 | 0.5° | 0.1° - 2.0° |
| **稳定时间** | 持续稳定的时间要求 | 3.0秒 | 1.0s - 10.0s |
| **自动保存** | 稳定后是否自动保存 | 启用(1) | 0=关闭, 1=开启 |

### 应用场景建议

**高精度应用：**
```shell
laser_cal_config 0.2 5.0 1    # 0.2°阈值，5秒稳定时间
```

**通用应用：**
```shell
laser_cal_config 0.5 3.0 1    # 0.5°阈值，3秒稳定时间
```

**快速校准：**
```shell
laser_cal_config 1.0 1.5 1    # 1.0°阈值，1.5秒稳定时间
```

## 📋 完整命令参考

### 基础校准命令

| 命令 | 功能 | 示例 |
|------|------|------|
| `laser_calibration on/off` | 进入/退出校准模式 | `laser_calibration on` |
| `laser_cal_config` | 配置监控参数 | `laser_cal_config 0.5 3.0 1` |
| `laser_cal_show` | 显示当前状态 | `laser_cal_show` |
| `laser_cal_wizard` | 显示校准向导 | `laser_cal_wizard` |

### 手动控制命令（兼容旧方式）

| 命令 | 功能 | 示例 |
|------|------|------|
| `laser_cal_offset` | 手动计算偏移 | `laser_cal_offset 0 0` |
| `laser_cal_save` | 手动保存参数 | `laser_cal_save` |
| `laser_cal_load` | 加载校准参数 | `laser_cal_load` |

## 🎯 校准最佳实践

### 准备工作
1. **目标选择**：选择3-5米外的明显目标点
2. **环境准备**：确保足够的照明和清晰的视线
3. **工具准备**：激光指示器或LED指示灯

### 校准过程
1. **位置确定**：将云台调整到能清楚看到目标的位置
2. **粗调**：先大致对准目标方向
3. **精调**：缓慢微调到精确对准
4. **稳定保持**：对准后保持稳定直到自动保存

### 验证测试
1. **零点测试**：`laser_angle 0 0` 应指向校准目标
2. **角度测试**：`laser_angle 30 0` 应指向右侧30°
3. **坐标测试**：使用实际坐标验证精度

## 🔍 故障排除

### 常见问题

**Q: 为什么进入校准模式后电机不动了？**
A: 这是正常的！新的校准模式会自动关闭电机，让你用手调整云台位置。

**Q: 角度一直不稳定，无法自动保存？**
A: 
- 检查云台是否固定牢固
- 适当放宽稳定阈值：`laser_cal_config 1.0 3.0 1`
- 确保环境没有振动干扰

**Q: 自动保存后角度还是不准？**
A: 
- 检查目标是否真的在正前方0°位置
- 重新校准，更仔细地对准目标
- 验证机械安装是否正确

**Q: 想要更快或更慢的校准速度？**
A: 调整稳定时间参数，如：
- 快速：`laser_cal_config 0.5 1.5 1`
- 精确：`laser_cal_config 0.3 5.0 1`

### 调试命令

```shell
# 查看实时角度
laser_cal_show

# 查看监控状态
laser_cal_config

# 手动强制保存
laser_cal_offset 0 0
laser_cal_save

# 重新加载校准参数
laser_cal_load
```

## 🎨 用户界面说明

### 实时显示格式

```
📐 Current angles: Yaw=5.23°, Pitch=-1.45° (Stable: 2.1s/3.0s)
```

- **Current angles**: 当前电机角度
- **Stable**: 当前稳定时间/要求稳定时间
- **✅ STABLE**: 已达到稳定要求

### 状态指示

- **🔓**: 校准模式已启用，电机关闭
- **📐**: 实时角度显示
- **⏱️**: 稳定性检测中
- **✅**: 自动校准完成
- **❌**: 操作失败或错误

## 💡 技术细节

### 监控线程工作流程

```
开始监控
    ↓
读取当前角度
    ↓
计算角度变化 → 变化 > 阈值？ → 是 → 重置计时器
    ↓                                    ↑
   否                                    |
    ↓                                    |
稳定时间足够？ → 否 ──────────────────────┘
    ↓
   是
    ↓
自动保存校准
    ↓
重置计时器
    ↓
继续监控
```

### 文件存储格式

校准参数保存在 `/laser_gimbal_calibration.conf`：

```
# Laser Gimbal Calibration Parameters
yaw_offset=5.23
pitch_offset=-1.45
yaw_reverse=1
pitch_reverse=0
gimbal_height=1.50
```

## 🚀 总结

新的自动校准功能极大简化了校准过程：

**原来需要：** 学习多个命令 → 手动微调 → 记住保存
**现在只需：** 开启校准 → 手动对准 → 自动完成

这使得即使是新用户也能轻松完成精确的校准工作！
