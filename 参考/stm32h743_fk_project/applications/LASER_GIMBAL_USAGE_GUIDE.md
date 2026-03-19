# 激光云台控制系统使用指南

## 概述

基于MS4010双电机的二维激光云台控制系统，支持通过输入距离和XY坐标来控制激光指向指定位置。

## 功能特点

- ✅ **坐标控制**: 输入距离和XY坐标，自动计算并控制激光指向
- ✅ **三维坐标支持**: 支持完整的三维坐标控制
- ✅ **直接角度控制**: 直接设置偏航角和俯仰角
- ✅ **平滑运动**: 可配置的运动速度和加速度参数
- ✅ **角度限制**: 可设置的安全角度限制范围
- ✅ **多种控制模式**: 直接控制、坐标控制、平滑运动、跟踪等
- ✅ **线程安全**: 完整的多线程安全设计
- ✅ **状态监控**: 实时状态反馈和统计信息

## 硬件连接

### 电机连接
- **偏航轴电机 (ID=1)**: 负责水平旋转 (左右转动)
- **俯仰轴电机 (ID=2)**: 负责垂直旋转 (上下转动)
- **通信接口**: RS485 (默认uart2, 115200波特率)

### 坐标系定义
```
        Y轴 (向右为正)
        ↑
        |
Z轴 ----+---→ X轴 (向前为正)
(向上为正)

偏航角 (Yaw): 绕Z轴旋转，向右为正 (-180° ~ +180°)
俯仰角 (Pitch): 绕Y轴旋转，向下为正 (-30° ~ +80°)
```

## 编译配置

在工程中添加以下文件：
- `laser_gimbal.h` - 头文件
- `laser_gimbal.c` - 实现文件  
- `laser_gimbal_test.c` - 测试和命令行接口

确保包含以下依赖：
- `drv_ms4010.h/c` - MS4010电机驱动
- RT-Thread操作系统
- 数学库支持 (`#include <math.h>`)

## 使用步骤

### 1. 系统初始化

```c
// 方法1: 使用命令行
laser_init

// 方法2: 程序调用
laser_gimbal_t gimbal;
ms4010_device_t *yaw_motor, *pitch_motor;

// 初始化电机管理器和获取电机设备
// ... (参考laser_gimbal_test.c中的laser_gimbal_system_init函数)

// 初始化云台系统
laser_gimbal_init(&gimbal, yaw_motor, pitch_motor, 1.5f); // 1.5米安装高度
```

### 2. 参数配置

```c
// 设置机械参数 (根据实际结构调整)
laser_gimbal_set_mechanical_params(&gimbal, 
    0.0f,    // 偏航零点偏移
    0.0f,    // 俯仰零点偏移
    false,   // 偏航轴不反向
    false);  // 俯仰轴不反向

// 设置运动参数
laser_gimbal_set_motion_params(&gimbal,
    45.0f,   // 最大偏航速度 45°/s
    30.0f,   // 最大俯仰速度 30°/s
    80.0f,   // 加速度 80°/s²
    0.8f);   // 平滑因子

// 设置角度限制
laser_gimbal_set_angle_limits(&gimbal,
    -120.0f, 120.0f,  // 偏航角度范围
    -20.0f, 60.0f);   // 俯仰角度范围
```

### 3. 控制方式

#### 方式1: 坐标控制 (推荐)

```c
// 命令行方式
laser_point 10.0 8.0 3.0  // 距离10m，前方8m，右侧3m

// 程序调用
laser_gimbal_point_to_coordinate(&gimbal, 10.0f, 8.0f, 3.0f);
```

#### 方式2: 三维坐标控制

```c
// 程序调用
laser_coordinate_t target = {8.0f, 3.0f, 2.0f}; // x=8m, y=3m, z=2m高度
laser_gimbal_point_to_3d_coordinate(&gimbal, target);
```

#### 方式3: 直接角度控制

```c
// 命令行方式
laser_angle 30.0 15.0  // 偏航30°，俯仰15°

// 程序调用
laser_gimbal_set_angle(&gimbal, 30.0f, 15.0f);
```

## 命令行接口

系统提供了完整的MSH命令行接口：

| 命令 | 参数 | 功能描述 |
|------|------|----------|
| `laser_init` | 无 | 初始化激光云台系统 |
| `laser_point` | `<distance> <x> <y>` | 指向指定坐标 |
| `laser_angle` | `<yaw> <pitch>` | 设置云台角度 |
| `laser_home` | 无 | 云台回零 |
| `laser_status` | 无 | 查看云台状态 |
| `laser_test` | `<basic\|scan\|precision>` | 运行测试程序 |
| `laser_stop` | 无 | 停止云台运动 |

### 使用示例

```shell
# 1. 初始化系统
msh> laser_init
I/laser_gimbal_test: === Initializing Laser Gimbal System ===
I/laser_gimbal_test: Scanning for motors...
I/ms4010: === MS4010 Quick Scan Complete: Total 2 motors found ===
I/laser_gimbal: Laser gimbal initialized successfully (height: 1.50m)

# 2. 指向目标点
msh> laser_point 5.0 4.0 2.0
Pointing to: distance=5.00m, x=4.00m, y=2.00m
I/laser_gimbal: Pointing to coordinate: (4.00, 2.00, 5.00) -> angle: (26.57°, -16.26°)

# 3. 直接角度控制
msh> laser_angle 45.0 20.0
Set angle: yaw=45.00°, pitch=20.00°

# 4. 查看状态
msh> laser_status
I/laser_gimbal_test: === Laser Gimbal Status ===
I/laser_gimbal_test: Current angle: yaw=45.00°, pitch=20.00°
I/laser_gimbal_test: State: IDLE
I/laser_gimbal_test: Statistics: Commands=2, Errors=0

# 5. 运行测试
msh> laser_test basic
I/laser_gimbal_test: === Starting Basic Function Test ===
```

## 测试程序

系统提供了三种测试模式：

### 1. 基本功能测试
```shell
laser_test basic
```
- 直接角度控制测试
- 坐标控制测试  
- 三维坐标控制测试
- 回零测试

### 2. 扫射模式测试
```shell
laser_test scan
```
- 水平扫射 (-30° ~ +30°)
- 垂直扫射 (-15° ~ +45°)

### 3. 精度测试
```shell
laser_test precision
```
- 多个预设目标点精度测试
- 角度计算验证

## 坐标计算原理

### 坐标转角度算法

对于目标点 (x, y, distance)：

1. **计算水平距离**:
   ```
   horizontal_distance = √(x² + y²)
   ```

2. **计算偏航角**:
   ```
   yaw = arctan2(y, x) * 180/π
   ```

3. **计算俯仰角**:
   ```
   vertical_distance = √(distance² - horizontal_distance²)
   height_diff = vertical_distance - gimbal_height
   pitch = arctan2(-height_diff, horizontal_distance) * 180/π
   ```

### 角度转坐标验证

用于验证计算准确性：
```c
horizontal_distance = distance * cos(pitch)
x = horizontal_distance * cos(yaw)  
y = horizontal_distance * sin(yaw)
z = gimbal_height - distance * sin(pitch)
```

## 性能参数

| 参数 | 规格 |
|------|------|
| 控制精度 | ±0.01° (MS4010电机精度) |
| 最大速度 | 可配置 (默认30-45°/s) |
| 加速度 | 可配置 (默认80°/s²) |
| 角度范围 | 偏航: ±120°, 俯仰: -20° ~ +60° |
| 控制距离 | 0.5m ~ 50m |
| 响应时间 | < 100ms |

## 故障排除

### 常见问题

1. **初始化失败**
   - 检查RS485连接和配置
   - 确认电机ID设置 (需要ID=1和ID=2)
   - 检查电机供电

2. **角度超限**
   ```
   E/laser_gimbal: Target angle out of limits: yaw=150.00°, pitch=70.00°
   ```
   - 调整角度限制参数
   - 检查目标坐标是否合理

3. **坐标超出范围**
   ```
   E/laser_gimbal: Target too far: 60.00m > 50.00m
   ```
   - 调整LASER_GIMBAL_MAX_DISTANCE参数
   - 使用合理的目标距离

4. **电机通信失败**
   - 检查RS485总线连接
   - 验证电机ID和波特率
   - 使用`ms4010_test status`检查电机状态

### 调试方法

1. **启用调试输出**:
   ```c
   #define DBG_LVL DBG_LOG  // 改为 DBG_INFO 或 DBG_LOG
   ```

2. **查看详细状态**:
   ```shell
   laser_status
   ms4010_test status
   ```

3. **单步测试**:
   ```shell
   laser_angle 0.0 0.0    # 回零
   laser_angle 10.0 0.0   # 小角度测试
   laser_point 3.0 3.0 0.0  # 简单坐标测试
   ```

## 进阶配置

### 自定义机械参数

根据你的实际机械结构调整参数：

```c
// 如果偏航轴安装方向相反
laser_gimbal_set_mechanical_params(&gimbal, 0.0f, 0.0f, true, false);

// 如果有零点偏移
laser_gimbal_set_mechanical_params(&gimbal, 5.0f, -2.0f, false, false);
```

### 自定义控制精度

```c
// 修改laser_gimbal.h中的参数
#define LASER_GIMBAL_MOTOR_ANGLE_SCALE  1000.0f  // 更高精度
```

### 添加自定义回调

```c
gimbal.on_target_reached = my_target_reached_callback;
gimbal.on_error = my_error_callback;
```

## 示例应用

### 应用1: 固定点巡检

```c
// 定义巡检点
laser_coordinate_t patrol_points[] = {
    {10.0f, 0.0f, 2.0f},   // 正前方
    {8.0f, 8.0f, 2.0f},    // 右前方  
    {0.0f, 10.0f, 2.0f},   // 右侧
    {-8.0f, 8.0f, 2.0f},   // 左前方
};

// 循环巡检
for (int i = 0; i < 4; i++) {
    laser_gimbal_point_to_3d_coordinate(&gimbal, patrol_points[i]);
    rt_thread_mdelay(3000); // 停留3秒
}
```

### 应用2: 区域扫描

```c
// 区域扫描 (前方5-15米，左右各30度)
for (float distance = 5.0f; distance <= 15.0f; distance += 2.0f) {
    for (float angle = -30.0f; angle <= 30.0f; angle += 5.0f) {
        float x = distance * cosf(LASER_GIMBAL_DEG_TO_RAD(angle));
        float y = distance * sinf(LASER_GIMBAL_DEG_TO_RAD(angle));
        laser_gimbal_point_to_coordinate(&gimbal, distance, x, y);
        rt_thread_mdelay(200);
    }
}
```

---

**注意**: 使用前请根据实际的机械结构和安装方式调整相关参数，确保安全运行。
