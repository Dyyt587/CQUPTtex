# 激光云台校准参数变量存储 - 使用示例

## 简化的校准流程

### 快速校准（手动）

```bash
# 1. 查看当前存储状态
laser_cal_storage

# 2. 进入校准模式
laser_calibration on

# 3. 手动调整云台到指向目标位置
laser_cal_set 0 0

# 4. 计算并保存偏移量（假设目标是原点 0,0）
laser_cal_offset 0 0
laser_cal_save

# 5. 退出校准模式
laser_calibration off

# 6. 验证校准效果
laser_cal_verify 0 0 1.0
```

### 自动校准（推荐）

```bash
# 1. 配置自动监控（0.5度阈值，3秒稳定时间，启用自动保存）
laser_cal_config 0.5 3000 1

# 2. 进入校准模式
laser_calibration on

# 3. 手动调整云台到目标位置并保持3秒不动
# 系统会自动检测稳定并计算偏移量，然后自动保存

# 4. 退出校准模式
laser_calibration off
```

## 存储管理命令

### 查看存储状态

```bash
laser_cal_storage
```

输出示例：
```
📊 校准参数存储状态:
  状态: ✅ 有效
  偏航偏移: 2.340°
  俯仰偏移: -1.250°
  偏航反向: 否
  俯仰反向: 是
  云台高度: 1.50m
  保存时间: 12345 ticks ago
```

### 重置存储

```bash
# 需要确认参数以防止误操作
laser_cal_reset confirm
```

## 系统行为

### 启动时

系统启动后会自动尝试加载校准参数：

- 如果存在有效参数：自动加载并应用
- 如果没有参数：使用默认值（偏移量为0）

### 校准参数生效

- **保存时立即生效**：`laser_cal_save` 后参数立即应用到云台控制中
- **重启后失效**：断电或重启后需要重新校准
- **实时应用**：所有云台运动都会应用当前的校准偏移量

## 故障排除

### 问题：校准参数没有生效

```bash
# 检查存储状态
laser_cal_storage

# 如果显示无效，重新校准
laser_calibration on
laser_cal_set 0 0
laser_cal_offset 0 0
laser_cal_save
laser_calibration off
```

### 问题：校准结果不准确

```bash
# 验证校准效果
laser_cal_verify 0 0 1.0

# 如果不准确，清除后重新校准
laser_cal_reset confirm
# 然后重新执行校准流程
```

### 问题：系统重启后校准丢失

这是正常现象，因为使用的是内存存储。系统重启后需要：

1. 重新进行校准，或
2. 考虑实现持久化存储（Flash/EEPROM）

## 优化建议

### 提高校准精度

1. **多点校准**：在不同位置进行多次校准取平均值
2. **细调步长**：使用 `laser_cal_adjust` 进行微调
3. **验证测试**：使用 `laser_cal_verify` 验证不同距离的精度

### 自动化程度

1. **使用自动监控**：配置合适的稳定阈值和时间
2. **批量测试**：编写脚本进行多点自动测试
3. **参数优化**：根据实际使用场景调整监控参数
