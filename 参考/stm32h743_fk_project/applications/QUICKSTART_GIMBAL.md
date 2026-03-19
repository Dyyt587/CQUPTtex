# 二维云台直线插补 - 快速开始

## 快速使用步骤

### 1. 编译项目
确保新添加的文件已加入编译：
- `gimbal_interpolation.h` - 头文件
- `gimbal_interpolation.c` - 实现文件  
- `gimbal_test.c` - 测试代码
- `gimbal_example.c` - 使用示例

### 2. 系统初始化
在终端中输入以下命令：
```bash
gimbal_init
```

### 3. 基本移动命令
```bash
# 移动到指定位置 (x, y, speed)
gimbal_move 30 45 25

# 查看当前位置
gimbal_pos

# 回到原点
gimbal_home
```

### 4. 测试轨迹
```bash
# 测试正方形轨迹
gimbal_square

# 测试对角线运动
gimbal_diagonal
```

### 5. 代码示例
```c
// 初始化
gimbal_interpolation_init(&gimbal_interp, &stepper_motor_1, &stepper_motor_2, 50);
gimbal_start_interpolation_timer();

// 直线运动
gimbal_g01_line(30.0f, 45.0f, 25.0f);  // 移动到(30,45)，速度25度/秒
gimbal_wait_motion_finish(0);           // 等待完成
```

## 参数说明

- **坐标单位**：度 (°)
- **速度单位**：度/秒 (°/s)  
- **插补周期**：50ms (可调)
- **脉冲比例**：3200脉冲/圈 (360度)

## 常用命令总结

| 命令 | 功能 | 示例 |
|------|------|------|
| `gimbal_init` | 初始化系统 | `gimbal_init` |
| `gimbal_move` | 移动到指定位置 | `gimbal_move 30 45 25` |
| `gimbal_pos` | 显示当前位置 | `gimbal_pos` |
| `gimbal_stop` | 停止当前运动 | `gimbal_stop` |
| `gimbal_home` | 回到原点 | `gimbal_home` |
| `gimbal_square` | 正方形轨迹测试 | `gimbal_square` |

## 故障排除

1. **初始化失败**：检查电机连接和驱动器状态
2. **运动不准确**：调整插补周期和速度参数
3. **无响应**：确保定时器已启动 (`gimbal_init`)

## 下一步

- 查看 `README_GIMBAL_INTERPOLATION.md` 了解详细功能
- 运行 `gimbal_interpolation_examples` 查看更多示例
- 根据需要调整参数和回调函数
