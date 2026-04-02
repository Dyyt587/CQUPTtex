# 二维云台直线插补使用说明

## 概述

本模块实现了二维云台的直线插补功能，支持精确的轨迹控制。系统采用时间片插补算法，能够实现平滑的直线运动。

## 文件结构

```
applications/
├── gimbal_interpolation.h    # 插补算法头文件
├── gimbal_interpolation.c    # 插补算法实现
├── gimbal_test.c            # 测试和示例代码
└── drv_emm_v5.h/.c          # 步进电机驱动(已有)
```

## 主要特性

- ✅ 直线插补算法
- ✅ 可配置进给速度
- ✅ 精确的时间控制
- ✅ 位置反馈
- ✅ 状态监控
- ✅ 回调函数支持
- 🔄 圆弧插补(待实现)

## 核心结构体

### 坐标点结构体
```c
typedef struct {
    float x;    // X轴坐标 (单位：度)
    float y;    // Y轴坐标 (单位：度)  
} point_t;
```

### 插补控制结构体
```c
typedef struct {
    interpolation_type_t type;     // 插补类型
    interpolation_state_t state;   // 插补状态
    point_t current_pos;           // 当前位置
    point_t target_pos;            // 目标位置
    stepper_motor_t *motor_x;      // X轴电机
    stepper_motor_t *motor_y;      // Y轴电机
    // ... 其他参数
} gimbal_interpolation_t;
```

## 使用方法

### 1. 系统初始化

```c
// 初始化插补系统
int ret = gimbal_interpolation_init(&gimbal_interp, 
                                   &stepper_motor_1,  // X轴电机
                                   &stepper_motor_2,  // Y轴电机
                                   50);               // 插补周期50ms

// 设置回调函数(可选)
gimbal_set_step_callback(&gimbal_interp, step_callback);
gimbal_set_finish_callback(&gimbal_interp, finish_callback);

// 启动插补定时器
gimbal_start_interpolation_timer();
```

### 2. 直线插补运动

#### 方法一：使用底层API
```c
// 设置直线插补参数
gimbal_set_line_interpolation(&gimbal_interp,
                             0.0f, 0.0f,      // 起点(x,y)
                             30.0f, 45.0f,    // 终点(x,y)  
                             25.0f);          // 进给速度(度/秒)

// 开始运动
gimbal_start_interpolation(&gimbal_interp);

// 等待完成
gimbal_wait_motion_finish(5000);  // 5秒超时
```

#### 方法二：使用G代码风格API
```c
// 直接移动到指定位置(G01命令)
gimbal_g01_line(30.0f, 45.0f, 25.0f);

// 等待完成
gimbal_wait_motion_finish(0);  // 无限等待
```

### 3. 状态监控

```c
// 获取当前状态
interpolation_state_t state = gimbal_get_interpolation_state(&gimbal_interp);

// 获取当前位置
point_t pos;
gimbal_get_current_position(&gimbal_interp, &pos);

// 检查是否完成
bool finished = gimbal_is_motion_finished(&gimbal_interp);
```

## 控制台命令

系统提供了丰富的控制台命令用于测试：

```bash
# 初始化系统
gimbal_init

# 移动到指定位置
gimbal_move 30 45 25    # 移动到(30,45)，速度25度/秒

# 测试正方形轨迹
gimbal_square

# 测试对角线运动  
gimbal_diagonal

# 回到原点
gimbal_home

# 显示当前位置
gimbal_pos

# 停止当前运动
gimbal_stop
```

## 配置参数

### 插补参数
```c
#define GIMBAL_INTERP_PERIOD     50       // 插补周期(ms)
#define GIMBAL_TEST_FEED_RATE    30.0f    // 默认进给速度(度/秒)
```

### 精度参数
```c
#define POSITION_EPSILON 0.01    // 位置比较精度
#define ANGLE_EPSILON 1e-6       // 角度比较精度
```

## 算法原理

### 直线插补算法

1. **参数计算**
   ```c
   float distance = sqrt((x2-x1)² + (y2-y1)²);
   float total_time = distance / feed_rate;
   uint32_t total_steps = total_time * 1000 / period_ms;
   ```

2. **位置计算**
   ```c
   float ratio = current_step / total_steps;
   next_x = start_x + ratio * (end_x - start_x);
   next_y = start_y + ratio * (end_y - start_y);
   ```

3. **电机控制**
   ```c
   uint32_t pulses = |delta_angle| * 3200 / 360;  // 脉冲数
   uint8_t direction = (delta_angle >= 0) ? 1 : 0; // 方向
   Emm_V5_Pos_Control(motor, direction, speed, acc, pulses, 1, 0);
   ```

## 测试示例

### 正方形轨迹测试
```c
void test_square_trajectory(void) {
    point_t points[] = {
        {0.0f, 0.0f},    {30.0f, 0.0f},
        {30.0f, 30.0f},  {0.0f, 30.0f},
        {0.0f, 0.0f}
    };
    
    for (int i = 1; i < 5; i++) {
        gimbal_set_line_interpolation(&gimbal_interp,
                                     points[i-1].x, points[i-1].y,
                                     points[i].x, points[i].y,
                                     30.0f);
        gimbal_start_interpolation(&gimbal_interp);
        gimbal_wait_motion_finish(10000);
    }
}
```

## 注意事项

1. **电机初始化**：确保步进电机已正确初始化和使能
2. **插补周期**：建议使用50-100ms的插补周期
3. **进给速度**：根据电机性能调整，避免过快导致丢步
4. **坐标系**：默认使用角度制，可根据需要修改
5. **安全限位**：实际使用时应添加限位检查

## 扩展功能

### 回调函数示例
```c
void step_callback(point_t current, point_t target) {
    printf("Moving: (%.2f,%.2f) -> (%.2f,%.2f)\n", 
           current.x, current.y, target.x, target.y);
}

void finish_callback(void) {
    printf("Motion completed!\n");
    // 执行后续动作
}
```

### 轨迹规划
```c
// 多段直线连接
typedef struct {
    point_t points[10];
    int count;
    float feed_rate;
} trajectory_t;

void execute_trajectory(trajectory_t *traj) {
    for (int i = 1; i < traj->count; i++) {
        gimbal_g01_line(traj->points[i].x, 
                       traj->points[i].y, 
                       traj->feed_rate);
        gimbal_wait_motion_finish(0);
    }
}
```

## 故障排除

1. **运动不平滑**：检查插补周期设置和电机加速度参数
2. **定位精度差**：调整位置精度参数和电机细分设置  
3. **运动异常**：检查电机接线和驱动器参数
4. **系统卡死**：检查互斥锁和线程同步

## 版本信息

- 当前版本：v1.0
- 更新日期：2025-07-25
- 作者：GitHub Copilot
- 兼容性：RT-Thread + STM32H743
