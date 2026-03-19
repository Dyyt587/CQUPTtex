# 激光云台绘制功能使用指南

## 功能概述

本模块实现了使用激光云台进行图形绘制的功能，支持以下图形：
- **矩形** - 通过四条边的连续点绘制
- **圆形** - 通过圆周上的连续点绘制  
- **正弦波** - 通过正弦函数采样点绘制

所有绘制都基于`laser_point`打点功能，通过连续的激光定位点形成图形轮廓。

## 绘制命令

### 1. 矩形绘制

```bash
laser_draw_rect <center_x> <center_y> <width> <height> <distance>
```

**参数说明**：
- `center_x`: 矩形中心X坐标 (米)
- `center_y`: 矩形中心Y坐标 (米)  
- `width`: 矩形宽度 (米)
- `height`: 矩形高度 (米)
- `distance`: 绘制距离 (米)

**使用示例**：
```bash
# 在5米距离处绘制中心在(2,1)，尺寸为3x2米的矩形
laser_draw_rect 2.0 1.0 3.0 2.0 5.0

# 绘制正方形
laser_draw_rect 0.0 0.0 2.0 2.0 4.0
```

**绘制过程**：
- 每条边使用20个点
- 按左下→右下→右上→左上→左下的顺序绘制
- 每个点停留100ms

### 2. 圆形绘制

```bash
laser_draw_circle <center_x> <center_y> <radius> <distance>
```

**参数说明**：
- `center_x`: 圆心X坐标 (米)
- `center_y`: 圆心Y坐标 (米)
- `radius`: 圆半径 (米)
- `distance`: 绘制距离 (米)

**使用示例**：
```bash
# 在6米距离处绘制中心在(3,2)，半径1.5米的圆
laser_draw_circle 3.0 2.0 1.5 6.0

# 绘制单位圆
laser_draw_circle 0.0 0.0 1.0 5.0
```

**绘制过程**：
- 圆周使用60个点
- 从0°开始逆时针绘制一圈
- 每个点停留80ms
- 每10个点显示进度

### 3. 正弦波绘制

```bash
laser_draw_sine <amplitude> <frequency> <x_start> <x_end> <y_offset> <distance> [phase]
```

**参数说明**：
- `amplitude`: 波幅 (米)
- `frequency`: 频率 (周期数)
- `x_start`: X轴起始位置 (米)
- `x_end`: X轴结束位置 (米)
- `y_offset`: Y轴偏移 (米)
- `distance`: 绘制距离 (米)
- `phase`: 相位偏移 (度，可选，默认0)

**数学公式**：
```
y = amplitude × sin(2π × frequency × t + phase) + y_offset
其中 t = (x - x_start) / (x_end - x_start)
```

**使用示例**：
```bash
# 基本正弦波：振幅1米，2个周期，X范围0-4米，Y偏移2米
laser_draw_sine 1.0 2.0 0.0 4.0 2.0 5.0

# 带相位的正弦波：相位偏移45度
laser_draw_sine 0.8 1.5 -2.0 2.0 1.0 6.0 45.0

# 高频正弦波
laser_draw_sine 0.5 5.0 0.0 3.0 0.0 4.0
```

**绘制过程**：
- 使用100个采样点
- 从x_start到x_end均匀采样
- 每个点停留60ms
- 每20个点显示进度

### 4. 演示模式

```bash
laser_draw_demo [distance]
```

**功能**：
- 自动演示所有绘制功能
- 包含矩形、圆形、两种正弦波
- 每个图形之间有1秒间隔

**使用示例**：
```bash
# 在默认5米距离演示
laser_draw_demo

# 在指定距离演示
laser_draw_demo 8.0
```

## 坐标系统

### 坐标定义
```
       Y (右方向)
       ↑
       |
       |
Z ←----+----→ X (前方向)
       |
       |
       ↓
   (左方向)
```

### 绘制平面
- 绘制在XY平面上进行
- Z坐标由distance参数确定
- 所有点的Z坐标相同，形成平面图形

### 角度对应
- X坐标 → 偏航角 (水平转动)
- Y坐标 → 俯仰角 (垂直转动)
- 坐标(0,0) → 激光指向正前方

## 使用流程

### 1. 准备工作
```bash
# 初始化系统
laser_init

# 校准云台 (如果需要)
laser_calibration on
# ... 校准过程 ...
laser_calibration off

# 回到home点
laser_home
```

### 2. 基础绘制
```bash
# 绘制简单矩形
laser_draw_rect 1.0 0.5 2.0 1.0 4.0

# 绘制圆形
laser_draw_circle 2.0 1.0 0.8 5.0

# 绘制正弦波
laser_draw_sine 0.6 1.0 0.0 3.0 1.5 6.0
```

### 3. 高级应用
```bash
# 复杂正弦波
laser_draw_sine 1.2 3.0 -1.5 1.5 0.0 7.0 90.0

# 大尺寸图形
laser_draw_rect 3.0 2.0 4.0 3.0 10.0

# 高精度圆形 (修改代码中的point_count)
```

## 技术参数

### 绘制精度
- **矩形**: 每边20个点，总计80个点
- **圆形**: 圆周60个点
- **正弦波**: 100个采样点

### 绘制速度
- **矩形**: 每点100ms，总耗时约8秒
- **圆形**: 每点80ms，总耗时约4.8秒  
- **正弦波**: 每点60ms，总耗时约6秒

### 尺寸限制
- **最小尺寸**: 受云台精度限制，建议>0.1米
- **最大尺寸**: 受云台角度范围限制
- **推荐距离**: 3-10米，平衡精度和范围

## 优化建议

### 1. 提高绘制质量
```c
// 增加点数 (在代码中修改)
int point_count = 40;  // 矩形每边40个点
int point_count = 120; // 圆形120个点
int point_count = 200; // 正弦波200个点
```

### 2. 调整绘制速度
```c
// 减少延时以提高速度
rt_thread_mdelay(50);  // 从100ms改为50ms

// 增加延时以提高稳定性
rt_thread_mdelay(150); // 从100ms改为150ms
```

### 3. 自定义图形
```c
// 基于现有函数开发新图形
static void laser_draw_triangle(float x1, float y1, float x2, float y2, 
                               float x3, float y3, float distance)
{
    // 绘制三角形的三条边
    // 类似矩形绘制逻辑
}
```

## 故障排除

### 常见问题

1. **图形变形**
   - 检查云台校准是否正确
   - 使用`laser_home_test`验证home点
   - 重新校准偏移参数

2. **绘制中断**
   - 检查参数范围是否超出云台限制
   - 确认电机通信正常
   - 使用`laser_status`查看状态

3. **精度不足**
   - 增加绘制点数
   - 减少绘制速度
   - 选择合适的绘制距离

4. **坐标偏移**
   - 确认坐标系统理解正确
   - 检查home点校准
   - 验证距离参数

### 调试命令
```bash
# 检查系统状态
laser_status

# 测试基本定位
laser_point 5.0 2.0 1.0

# 验证home点
laser_home_test

# 显示校准信息
laser_cal_show
```

## 扩展应用

### 1. 艺术创作
- 组合多个基本图形
- 使用不同参数创建复杂图案
- 实现动态绘制效果

### 2. 教学演示  
- 数学函数可视化
- 几何图形教学
- 物理概念演示

### 3. 工程应用
- 激光标记系统
- 轮廓检测校准
- 精度测试工具

### 4. 程序化绘制
```bash
# 批量绘制脚本示例
laser_draw_circle 0.0 0.0 0.5 5.0    # 内圆
laser_draw_circle 0.0 0.0 1.0 5.0    # 中圆  
laser_draw_circle 0.0 0.0 1.5 5.0    # 外圆
```

通过这些功能，您可以实现丰富的激光绘制应用，从简单的几何图形到复杂的数学函数可视化。
