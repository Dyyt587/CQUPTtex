# STM32云台工程代码总结（参考工程：stm32h743_fk_project）

## 1. 工程定位与运行环境
- 代码基于 RT-Thread，主控平台为 STM32H743。
- 云台核心由双轴电机驱动：偏航轴（Yaw）与俯仰轴（Pitch）。
- 主要业务代码位于 applications 目录；HAL、BSP、RT-Thread 内核和中间件位于 board、libraries、rt-thread 等目录。

## 2. 云台软件总体架构
工程可划分为 5 个层次：
1. 设备驱动层：
- MS4010 电机 RS485 驱动（drv_ms4010.c/.h）
- HiPNUC IMU 解码（hipnuc_dec.c/.h）
- 串口视觉数据解析（usart_dataparsing.c/.h）

2. 运动控制层：
- 云台基础控制（laser_gimbal.c/.h）
- 提供角度控制、速度控制、坐标指向、限位、回零、电机使能与停机。

3. 视觉追踪层：
- 视觉追踪器（laser_gimbal_vision_tracker.c/.h）
- 通过像素误差到角度误差映射，并使用 APID 进行闭环调节。

4. 系统集成层：
- 系统初始化与测试入口（laser_gimbal_test.c、laser_gimbal_vision_test.c、main.c）
- 负责模块装配、参数下发、模式切换与运行控制。

5. 命令与调参层：
- MSH 命令导出（MSH_CMD_EXPORT）用于在线调试，如云台角度、速度、校准、视觉参数、补偿增益调整。

## 3. 关键执行流程
### 3.1 主程序初始化流程
main.c 中完成 GPIO 初始化后，核心流程为：
- 调用 laser_gimbal_system_init 完成云台系统装配。
- 调用 vision_init 初始化视觉追踪器。
- 设置追踪模式（连续模式），并在循环中使能追踪。

该流程表明系统采用“先底层执行器，再上层感知追踪”的启动顺序，便于定位初始化失败点。

### 3.2 云台初始化流程
laser_gimbal_system_init 的关键步骤：
- 初始化 ms4010_manager（串口与总线管理）。
- 快速扫描电机并获取 ID1/ID2（分别对应 Yaw/Pitch）。
- 调用 laser_gimbal_init 建立云台对象与参数。
- 设置机械零偏、运动参数、角度限位。
- 使能电机并执行回零。

该流程体现了“硬件探测 -> 参数绑定 -> 安全使能 -> 回零建立基准”的工程链路。

## 4. 控制策略总结
### 4.1 几何解算与角度指向
laser_gimbal_coordinate_to_angle 中通过 atan2 完成坐标到姿态角转换。
- yaw 由 x/z 求解
- pitch 由 y/z 求解
- 结果经角度归一化和限位检查后执行

对应接口包括：
- laser_gimbal_point_to_coordinate
- laser_gimbal_point_to_3d_coordinate
- laser_gimbal_set_angle

### 4.2 位置与速度双模式控制
同一云台对象支持两类执行模式：
- 位置模式：按目标角度控制，适合指向和跟踪收敛。
- 速度模式：按角速度连续旋转，适合搜索和快速补偿。

速度模式接口：
- laser_gimbal_set_yaw_speed
- laser_gimbal_set_pitch_speed
- laser_gimbal_set_angular_velocity
- laser_gimbal_stop_velocity_control

### 4.3 视觉闭环追踪
vision_tracker_thread_entry 以较高频率运行（2ms 延时，约 500Hz），核心逻辑是：
- 读取目标像素坐标与置信度。
- 像素误差转角度误差。
- APID 输出 yaw/pitch 控制量。
- 在追踪状态下调用 vision_tracker_apply_control 驱动云台。

同时存在目标历史滤波、死区、丢失超时、自动搜索等机制，提升目标抖动场景下的稳定性。

### 4.4 IMU补偿与前馈增强
工程将 IMU 角速度/加速度引入追踪控制：
- 读取 hipnuc_raw 中的 gyr 与 acc 数据。
- 对 yaw 方向叠加角速度/角度变化率/加速度相关补偿项。
- 提供 acc_kp、ang_kp、vel_kp 在线增益调节命令。

该机制在载体突然加减速、抖动场景下可降低视觉闭环滞后。

## 5. 通信与数据通路
### 5.1 视觉输入协议
usart_dataparsing.c 实现基于帧头尾的串口协议解析：
- 帧格式：@data1,data2,data3,#
- 数据解析后调用 vision_tracker_input_target
- 典型字段对应为 pixel_x, pixel_y, confidence

### 5.2 电机控制通信
drv_ms4010.c 封装 MS4010 协议帧与收发，含：
- 命令打包、响应解析、校验
- 超时管理
- 总线互斥，避免多电机并发冲突

## 6. 并发与线程安全设计
- 云台和追踪器均使用互斥锁（gimbal_mutex、tracker_mutex）保护共享状态。
- 追踪线程、校准监控线程、陀螺补偿线程分工独立。
- 多电机共享 RS485 总线，使用 bus_mutex 串行化访问。

该设计降低了并发条件下的数据竞争风险，但需要统一锁粒度与超时策略以避免高负载下的控制抖动。

## 7. 工程优点与风险点
### 7.1 优点
- 分层明确：驱动、控制、追踪、测试分离。
- 调试友好：大量 MSH 命令支持在线验证与参数整定。
- 控制完整：具备坐标指向、角度控制、速度控制、视觉追踪、IMU补偿。

### 7.2 风险点
- 部分互斥保护代码被注释，可能引入竞态。
- 主循环反复使能追踪的调用方式可能造成状态机抖动。
- 视觉线程中补偿项叠加较多，需结合实测做抗饱和与抗积分风up验证。

## 8. 对论文“软件设计”章节的可用结论
可直接提炼为以下论文章节点：
- 软件分层架构与任务划分
- 初始化流程与状态机设计
- 视觉闭环与 IMU 前馈融合策略
- 串口协议与电机总线通信机制
- 线程同步、异常处理与可维护性分析

上述内容能够将“软件设计”从概念描述提升为“基于真实工程代码的可复现实现说明”。
