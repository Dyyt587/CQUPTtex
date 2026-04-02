# 激光云台电机轴分配修改说明

## 修改概述

根据要求，将激光云台的电机轴分配明确定义为：
- **1号电机 (ID=1)** → **X轴** (对应偏航轴，水平旋转)
- **2号电机 (ID=2)** → **Y轴** (对应俯仰轴，垂直旋转)

## 修改内容

### 1. 测试文件变量名更新

**文件**: `laser_gimbal_test.c`

**变量重命名**:
```c
// 修改前
static ms4010_device_t *g_yaw_motor = RT_NULL;
static ms4010_device_t *g_pitch_motor = RT_NULL;

// 修改后  
static ms4010_device_t *g_x_motor = RT_NULL;      // X轴电机 (ID=1, 对应偏航轴)
static ms4010_device_t *g_y_motor = RT_NULL;      // Y轴电机 (ID=2, 对应俯仰轴)
```

**电机获取**:
```c
// 修改前
g_yaw_motor = ms4010_manager_get_motor(&g_motor_manager, 1);    // 偏航轴
g_pitch_motor = ms4010_manager_get_motor(&g_motor_manager, 2);  // 俯仰轴

// 修改后
g_x_motor = ms4010_manager_get_motor(&g_motor_manager, 1);    // X轴电机 (偏航轴)
g_y_motor = ms4010_manager_get_motor(&g_motor_manager, 2);    // Y轴电机 (俯仰轴)
```

**初始化调用**:
```c
// 修改前
result = laser_gimbal_init(&g_laser_gimbal, g_yaw_motor, g_pitch_motor, 0.2f);

// 修改后
result = laser_gimbal_init(&g_laser_gimbal, g_x_motor, g_y_motor, 0.2f);
```

### 2. 头文件注释更新

**文件**: `laser_gimbal.h`

**结构体成员注释**:
```c
// 修改前
ms4010_device_t *yaw_motor;         // 偏航轴电机 (水平旋转)
ms4010_device_t *pitch_motor;       // 俯仰轴电机 (垂直旋转)

// 修改后
ms4010_device_t *yaw_motor;         // X轴电机 (偏航轴, 水平旋转, ID=1)
ms4010_device_t *pitch_motor;       // Y轴电机 (俯仰轴, 垂直旋转, ID=2)
```

**机械参数注释**:
```c
// 修改前
float yaw_offset;                   // 偏航轴零点偏移 (度)
float pitch_offset;                 // 俯仰轴零点偏移 (度)
bool yaw_reverse;                   // 偏航轴反向
bool pitch_reverse;                 // 俯仰轴反向

// 修改后
float yaw_offset;                   // X轴(偏航)零点偏移 (度)
float pitch_offset;                 // Y轴(俯仰)零点偏移 (度)
bool yaw_reverse;                   // X轴反向
bool pitch_reverse;                 // Y轴反向
```

**函数参数注释**:
```c
// 修改前
* @param yaw_motor 偏航轴电机设备指针
* @param pitch_motor 俯仰轴电机设备指针

// 修改后
* @param yaw_motor X轴电机设备指针 (ID=1, 偏航轴, 水平旋转)
* @param pitch_motor Y轴电机设备指针 (ID=2, 俯仰轴, 垂直旋转)
```

## 电机轴对应关系

| 电机ID | 轴名称 | 内部变量名 | 运动方向 | 控制角度 | 物理描述 |
|--------|--------|------------|----------|----------|----------|
| 1      | X轴    | yaw_motor  | 水平旋转 | 偏航角   | 左右转动 |
| 2      | Y轴    | pitch_motor| 垂直旋转 | 俯仰角   | 上下转动 |

## 坐标系统说明

### 标准坐标系
- **X轴正方向**: 向右
- **Y轴正方向**: 向上  
- **Z轴正方向**: 向前（激光指向）

### 角度定义
- **X轴角度 (偏航角)**:
  - 0° = 正前方
  - +90° = 向右
  - -90° = 向左
  - ±180° = 正后方

- **Y轴角度 (俯仰角)**:
  - 0° = 水平
  - +90° = 向上
  - -90° = 向下

## 使用示例

### 基本控制命令

```bash
# 控制激光指向指定位置 (X=1m, Y=2m, Z=3m)
laser_point 1.0 2.0 3.0

# 直接设置角度 (X轴=30°, Y轴=15°)
laser_angle 30.0 15.0

# 校准模式下设置电机角度
laser_cal_set 45.0 -10.0    # X轴45°, Y轴-10°
```

### 校准流程

```bash
# 1. 进入校准模式
laser_calibration on

# 2. 调整X轴电机 (1号电机) 和Y轴电机 (2号电机)
laser_cal_set 0 0           # 设置为基准位置

# 3. 微调 (如果需要)
laser_cal_adjust 1.0 -0.5   # X轴+1°, Y轴-0.5°

# 4. 保存校准 (当激光指向目标位置时)
laser_cal_offset 0 0        # 期望指向0°,0°
laser_cal_save              # 保存校准参数

# 5. 退出校准模式
laser_calibration off
```

## 兼容性说明

- **API兼容**: 所有函数接口保持不变
- **参数含义**: 函数参数名称未改变，但注释更明确
- **内部实现**: 电机控制逻辑完全相同
- **配置文件**: 校准参数格式保持一致

## 注意事项

1. **电机ID固定**: 
   - 必须确保1号电机连接到X轴（偏航轴）
   - 必须确保2号电机连接到Y轴（俯仰轴）

2. **方向设置**: 
   - 根据实际机械结构设置 `yaw_reverse` 和 `pitch_reverse`
   - 确保角度增加方向符合右手坐标系

3. **校准重要性**:
   - 电机安装角度可能与理论值有偏差
   - 必须进行校准以确保指向精度

4. **调试建议**:
   - 使用 `laser_cal_show` 查看当前电机角度
   - 使用 `laser_cal_verify` 验证校准效果
   - 逐步调整直到达到满意精度

## 后续扩展

如果需要增加更多轴或改变电机分配，可以考虑：

1. **三轴系统**: 增加Z轴电机（如焦距控制）
2. **灵活配置**: 支持运行时配置电机ID映射
3. **多云台**: 支持多个云台系统并行工作
4. **自动检测**: 自动检测电机连接和轴向映射
