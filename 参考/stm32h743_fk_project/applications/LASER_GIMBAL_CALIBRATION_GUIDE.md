# 激光云台角度校准指南

## 概述

由于电机安装角度可能不完全一致，激光云台系统提供了完整的角度偏移校准功能。通过校准，可以确保云台的逻辑角度（如0°正前方）与实际物理指向完全一致。

## 校准原理

### 坐标系定义
```
        Y轴 (向右为正)
        ↑
        |
Z轴 ----+---→ X轴 (向前为正)
(向上为正)

偏航角 (Yaw): 绕Z轴旋转，向右为正 (-180° ~ +180°)
俯仰角 (Pitch): 绕Y轴旋转，向下为正 (-90° ~ +90°)
```

### 角度转换关系
```
实际输出角度 = 电机原始角度 + 偏移量
偏移量 = 期望角度 - 电机原始角度
```

## 校准步骤

### 准备工作

1. **硬件准备**
   - 确保云台已正确安装并通电
   - 准备激光指示器或其他指向设备
   - 在适当距离（3-5米）放置目标板或墙面

2. **软件准备**
   ```shell
   # 初始化激光云台系统
   msh> laser_init
   ```

### 第一步：进入校准模式

```shell
# 进入校准模式 - 启用直接电机控制
msh> laser_calibration on
```

**说明**: 校准模式下，系统会绕过角度偏移和限位检查，直接控制电机原始角度。

### 第二步：手动调整云台位置

#### 方法1：绝对角度设置
```shell
# 设置电机到特定原始角度
msh> laser_cal_set 0.0 0.0    # 设置偏航0°，俯仰0°
```

#### 方法2：增量调整（推荐）
```shell
# 查看当前角度
msh> laser_cal_show

# 进行小幅调整
msh> laser_cal_adjust 2.0 -1.0    # 偏航+2°，俯仰-1°
msh> laser_cal_adjust 0.5 0.2     # 继续精细调整
msh> laser_cal_adjust -0.1 0.0    # 非常精细的调整
```

**调整技巧**：
- 先进行大幅调整接近目标位置
- 再进行小幅调整精确对准
- 每次调整后观察激光点位置
- 重复调整直到激光指向完全符合预期

### 第三步：设置参考位置

当云台指向理想的0°位置（通常是正前方水平）时：

```shell
# 将当前位置设置为0°, 0°参考点
msh> laser_cal_offset 0.0 0.0

# 或设置为其他参考角度
msh> laser_cal_offset 45.0 10.0   # 设置为45°偏航，10°俯仰
```

**重要**: 这一步计算并设置了角度偏移量，确保后续的逻辑角度与实际指向一致。

### 第四步：保存校准参数

```shell
# 保存到默认文件
msh> laser_cal_save

# 或保存到指定文件
msh> laser_cal_save /my_calibration.txt
```

### 第五步：退出校准模式

```shell
# 退出校准模式，恢复正常操作
msh> laser_calibration off
```

### 第六步：验证校准结果

```shell
# 测试基本角度控制
msh> laser_angle 0.0 0.0      # 应指向正前方
msh> laser_angle 45.0 0.0     # 应指向右侧45°
msh> laser_angle -30.0 20.0   # 应指向左侧30°，向下20°

# 测试坐标控制
msh> laser_point 5.0 5.0 0.0  # 应指向前方5米处
```

## 校准命令参考

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

## 校准示例

### 完整校准流程示例

```shell
# 1. 进入校准模式
msh> laser_calibration on
I/laser_gimbal: Calibration mode ENABLED - Direct motor control active

# 2. 从已知位置开始
msh> laser_cal_set 0.0 0.0
I/laser_gimbal: Motor angles set: Yaw=0.00°, Pitch=0.00°

# 3. 观察激光点位置，进行调整
msh> laser_cal_adjust 3.2 -1.8    # 激光点偏左下，向右上调整
I/laser_gimbal: Adjusting angles: Yaw +3.20° -> 3.20°, Pitch -1.80° -> -1.80°

# 4. 继续精细调整
msh> laser_cal_adjust 0.3 0.1     # 再微调
I/laser_gimbal: Adjusting angles: Yaw +0.30° -> 3.50°, Pitch +0.10° -> -1.70°

# 5. 检查当前状态
msh> laser_cal_show
=== Current Motor Raw Angles ===
Yaw motor:   3.50°
Pitch motor: -1.70°

# 6. 当激光完全指向正前方时，设置为参考点
msh> laser_cal_offset 0.0 0.0
I/laser_gimbal: Calibration completed!
I/laser_gimbal:   Target angles: Yaw=0.00°, Pitch=0.00°
I/laser_gimbal:   Motor angles: Yaw=3.50°, Pitch=-1.70°
I/laser_gimbal:   Calculated offsets: Yaw=-3.50°, Pitch=1.70°

# 7. 保存校准参数
msh> laser_cal_save
I/laser_gimbal: Calibration parameters saved to: /calibration/laser_gimbal_cal.txt

# 8. 退出校准模式
msh> laser_calibration off
I/laser_gimbal: Calibration mode DISABLED - Normal operation resumed

# 9. 验证校准结果
msh> laser_angle 0.0 0.0          # 应指向正前方
msh> laser_angle 30.0 0.0         # 应指向右侧30°
```

### 处理安装反向的情况

如果电机安装方向与预期相反：

```shell
# 1. 进入校准模式并测试
msh> laser_calibration on
msh> laser_cal_set 10.0 0.0       # 设置偏航+10°

# 2. 如果云台向左转而不是向右转，说明偏航轴反向
#    在程序中设置反向标志
laser_gimbal_set_mechanical_params(&gimbal, 0.0f, 0.0f, true, false);

# 3. 如果俯仰轴也反向，设置俯仰轴反向
laser_gimbal_set_mechanical_params(&gimbal, 0.0f, 0.0f, true, true);

# 4. 重新进行校准流程
```

## 高级功能

### 自动加载校准参数

在系统初始化时自动加载校准参数：

```c
// 在laser_gimbal_system_init()中添加
result = laser_gimbal_load_calibration(&g_laser_gimbal, RT_NULL);
if (result == RT_EOK)
{
    LOG_I("Calibration parameters loaded automatically");
}
```

### 多组校准参数

为不同应用场景保存多组校准参数：

```shell
# 保存不同场景的校准
msh> laser_cal_save /cal_indoor.txt     # 室内校准
msh> laser_cal_save /cal_outdoor.txt    # 室外校准

# 根据需要加载
msh> laser_cal_load /cal_outdoor.txt
```

### 校准精度验证

验证校准精度的方法：

```shell
# 测试已知角度点
msh> laser_angle 0.0 0.0      # 记录激光点位置A
msh> laser_angle 90.0 0.0     # 记录激光点位置B
# 测量A、B两点夹角，应该接近90°

# 测试坐标控制精度
msh> laser_point 5.0 5.0 0.0  # 指向前方5米，右侧5米
# 实测距离和角度，验证坐标计算准确性
```

## 故障排除

### 常见问题

1. **校准模式无法进入**
   ```
   解决：确保激光云台系统已正确初始化
   检查：laser_status 查看系统状态
   ```

2. **角度调整无效果**
   ```
   解决：检查电机连接和通信
   检查：ms4010_test status 查看电机状态
   ```

3. **校准后角度仍不准确**
   ```
   解决：重复校准流程，确保参考位置精确
   检查：机械安装是否松动
   ```

4. **保存校准参数失败**
   ```
   解决：检查文件系统是否正常挂载
   尝试：使用不同的文件路径
   ```

### 调试方法

1. **启用详细日志**
   ```c
   #define DBG_LVL DBG_INFO  // 在laser_gimbal.c中
   ```

2. **手动检查偏移值**
   ```shell
   msh> laser_cal_show  # 查看当前偏移值是否合理
   ```

3. **步进测试**
   ```shell
   # 逐步测试各个角度
   msh> laser_angle 0 0
   msh> laser_angle 15 0
   msh> laser_angle 30 0
   # 观察每步的准确性
   ```

## 最佳实践

1. **定期重新校准**
   - 建议每月或在环境变化后重新校准
   - 机械部件松动或更换后必须重新校准

2. **备份校准参数**
   ```shell
   # 定期备份校准文件
   cp /calibration/laser_gimbal_cal.txt /backup/
   ```

3. **多点验证**
   - 不仅测试0°位置，还要测试多个角度点
   - 覆盖云台的整个工作范围

4. **环境因素考虑**
   - 温度变化可能影响机械精度
   - 安装基座的稳定性很重要

---

通过正确的校准，激光云台系统可以实现高精度的角度控制和坐标指向功能。
