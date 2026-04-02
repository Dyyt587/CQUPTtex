# 激光绘制功能快速测试脚本

## 基本功能测试

### 1. 系统初始化测试
```bash
# 初始化激光云台系统
laser_init

# 检查系统状态
laser_status

# 回到home点
laser_home

# 测试home点功能
laser_home_test
```

### 2. 基础绘制测试

#### 矩形绘制测试
```bash
# 小矩形测试 (2x1米，距离5米)
laser_draw_rect 1.0 0.5 2.0 1.0 5.0

# 正方形测试
laser_draw_rect 0.0 0.0 1.5 1.5 4.0

# 大矩形测试
laser_draw_rect 2.5 1.5 3.0 2.0 6.0
```

#### 圆形绘制测试  
```bash
# 小圆测试 (半径0.5米)
laser_draw_circle 1.0 1.0 0.5 4.0

# 单位圆测试
laser_draw_circle 0.0 0.0 1.0 5.0

# 大圆测试 (半径2米)
laser_draw_circle 3.0 2.0 2.0 8.0
```

#### 正弦波绘制测试
```bash
# 基础正弦波 (1个周期)
laser_draw_sine 1.0 1.0 0.0 2.0 1.0 5.0

# 高频正弦波 (3个周期)
laser_draw_sine 0.8 3.0 -1.0 2.0 2.0 6.0

# 带相位正弦波 (相位90度)
laser_draw_sine 1.2 2.0 0.0 3.0 0.0 5.0 90.0

# 低频正弦波 (0.5个周期)
laser_draw_sine 1.5 0.5 -2.0 2.0 1.5 7.0
```

### 3. 演示模式测试
```bash
# 运行完整演示 (默认5米距离)
laser_draw_demo

# 运行远距离演示 (8米距离)  
laser_draw_demo 8.0
```

## 进阶测试场景

### 1. 精度测试
```bash
# 微小图形测试 (测试精度下限)
laser_draw_rect 0.0 0.0 0.2 0.2 3.0
laser_draw_circle 0.0 0.0 0.1 3.0

# 巨大图形测试 (测试角度范围)
laser_draw_rect 5.0 3.0 6.0 4.0 15.0
laser_draw_circle 4.0 2.0 3.0 12.0
```

### 2. 边界条件测试
```bash
# 负坐标测试
laser_draw_rect -2.0 -1.0 1.0 0.8 5.0
laser_draw_circle -1.5 -1.5 0.8 6.0
laser_draw_sine 0.5 2.0 -3.0 -1.0 -1.0 5.0

# 极端参数测试
laser_draw_sine 2.0 5.0 0.0 1.0 3.0 10.0 180.0
```

### 3. 组合图形测试
```bash
# 同心圆
laser_draw_circle 2.0 1.0 0.5 6.0
laser_draw_circle 2.0 1.0 1.0 6.0  
laser_draw_circle 2.0 1.0 1.5 6.0

# 嵌套矩形
laser_draw_rect 1.0 1.0 1.0 1.0 5.0
laser_draw_rect 1.0 1.0 2.0 2.0 5.0
laser_draw_rect 1.0 1.0 3.0 3.0 5.0

# 正弦波组合
laser_draw_sine 0.8 1.0 0.0 4.0 2.0 6.0 0.0    # 基波
laser_draw_sine 0.4 2.0 0.0 4.0 2.0 6.0 0.0    # 二次谐波  
laser_draw_sine 0.2 3.0 0.0 4.0 2.0 6.0 90.0   # 三次谐波带相位
```

## 故障诊断测试

### 1. 参数错误测试
```bash
# 这些命令应该返回错误信息
laser_draw_rect 1.0 1.0 -1.0 1.0 5.0  # 负宽度
laser_draw_circle 1.0 1.0 -0.5 5.0     # 负半径
laser_draw_sine 1.0 2.0 2.0 1.0 1.0 5.0 # x_end < x_start
```

### 2. 系统状态测试
```bash
# 检查各种状态
laser_status                # 系统状态
laser_cal_show             # 校准状态
laser_cal_storage          # 校准存储状态
```

### 3. 基础功能验证
```bash
# 验证基础定位功能
laser_point 5.0 2.0 1.0    # 指向具体坐标
laser_angle 30.0 15.0      # 设置具体角度
laser_home                 # 回零测试
```

## 性能测试

### 1. 连续绘制测试
```bash
# 连续绘制多个图形 (测试稳定性)
laser_draw_rect 1.0 1.0 1.0 1.0 5.0
laser_draw_circle 2.0 1.0 0.5 5.0
laser_draw_sine 0.5 2.0 0.0 2.0 -1.0 5.0
laser_draw_rect -1.0 -1.0 1.5 1.0 5.0
```

### 2. 速度测试
```bash
# 记录每个图形的绘制时间
# 矩形：预期约8秒 (80点 × 100ms)
laser_draw_rect 2.0 1.0 2.0 1.5 5.0

# 圆形：预期约4.8秒 (60点 × 80ms)  
laser_draw_circle 1.0 0.0 1.0 5.0

# 正弦波：预期约6秒 (100点 × 60ms)
laser_draw_sine 1.0 2.0 0.0 3.0 1.0 5.0
```

## 校准验证测试

### 1. 校准前测试
```bash
# 重置校准参数
laser_cal_reset confirm

# 测试未校准状态下的绘制
laser_draw_rect 2.0 1.0 2.0 1.0 5.0
```

### 2. 校准后测试  
```bash
# 执行校准流程
laser_calibration on
# 手动调整到准确位置
laser_cal_offset 0 0
laser_cal_save
laser_calibration off

# 测试校准后的绘制精度
laser_draw_rect 2.0 1.0 2.0 1.0 5.0
laser_home_test
```

## 预期结果

### 正常输出示例
```
🔳 Drawing rectangle: center(2.0,1.0), size(2.0x1.0), distance=5.0m
Drawing edge 1...
Drawing edge 2...  
Drawing edge 3...
Drawing edge 4...
✅ Rectangle drawing completed

⭕ Drawing circle: center(1.0,0.0), radius=1.0, distance=5.0m
Progress: 0.0% (0.0°)
Progress: 16.7% (60.0°)
...
✅ Circle drawing completed

〰️ Drawing sine wave: A=1.0, f=2.0, φ=0.0°, x=[0.0,3.0], y_offset=1.0
Progress: 0.0% - Point(0.00, 1.00)
Progress: 20.0% - Point(0.60, 1.95)
...
✅ Sine wave drawing completed
```

### 错误信息示例
```
❌ Error: Width, height, and distance must be positive
❌ Failed to point to (2.50, 1.75)
❌ Error: x_end must be greater than x_start
```

## 测试清单

- [ ] 系统初始化正常
- [ ] Home点功能正常
- [ ] 矩形绘制正常
- [ ] 圆形绘制正常  
- [ ] 正弦波绘制正常
- [ ] 演示模式正常
- [ ] 参数验证正常
- [ ] 错误处理正常
- [ ] 校准功能正常
- [ ] 性能指标达标

通过这些测试，可以全面验证激光绘制功能的正确性和稳定性。
