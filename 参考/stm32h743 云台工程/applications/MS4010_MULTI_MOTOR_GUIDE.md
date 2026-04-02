# MS4010多电机控制系统使用指南

## 功能概述

MS4010驱动现已支持在单个RS485总线上控制多个电机，通过不同的ID号区分不同电机。系统提供了管理器架构来统一管理多个电机设备。

## 系统架构

### 1. 管理器架构
- **ms4010_manager_t**: 多电机管理器，负责RS485总线管理和电机设备管理
- **ms4010_device_t**: 单个电机设备，每个电机有独立的ID和状态
- **共享总线**: 多个电机共享同一个RS485总线，通过总线互斥锁防止冲突

### 2. 主要特性
- 支持最多16个电机同时控制
- 自动总线仲裁和互斥保护
- 统一的状态管理和错误处理
- 兼容原有单电机接口

## 快速开始

### 1. 初始化多电机系统
```bash
# 初始化管理器
ms4010_multi init

# 添加电机到系统 (ID范围: 1-255)
ms4010_multi add 1    # 添加ID为1的电机
ms4010_multi add 2    # 添加ID为2的电机
ms4010_multi add 3    # 添加ID为3的电机

# 查看当前电机列表
ms4010_multi list
```

### 2. 基本电机控制
```bash
# 控制特定电机
ms4010_multi motor 1 on           # 开启1号电机
ms4010_multi motor 1 pos 9000     # 1号电机移动到90度
ms4010_multi motor 2 speed 1000   # 2号电机以1000dps速度运行
ms4010_multi motor 1 off          # 关闭1号电机

# 查看电机状态
ms4010_multi motor 1 status       # 查看1号电机状态
ms4010_multi status               # 查看所有电机状态
```

### 3. 系统测试
```bash
# 基础通信测试
ms4010_multi test_basic

# 同步控制测试
ms4010_multi test_sync

# 性能测试
ms4010_multi test_performance

# 启动自动测试
ms4010_multi start_auto

# 停止自动测试
ms4010_multi stop_auto
```

## 详细命令参考

### 系统管理命令

#### 初始化和反初始化
```bash
ms4010_multi init      # 初始化多电机管理器
ms4010_multi deinit    # 反初始化管理器
```

#### 电机管理
```bash
ms4010_multi add <id>        # 添加电机 (ID: 1-255)
ms4010_multi remove <id>     # 移除电机
ms4010_multi list            # 列出所有电机
ms4010_multi status          # 获取所有电机状态
```

### 电机控制命令

#### 基本控制
```bash
ms4010_multi motor <id> on           # 开启电机
ms4010_multi motor <id> off          # 关闭电机
ms4010_multi motor <id> status       # 查看电机状态
```

#### 位置控制
```bash
ms4010_multi motor <id> pos <angle>  # 位置控制
# 示例:
ms4010_multi motor 1 pos 0      # 移动到0度
ms4010_multi motor 1 pos 9000   # 移动到90度
ms4010_multi motor 1 pos -18000 # 移动到-180度
```

#### 速度控制
```bash
ms4010_multi motor <id> speed <dps>  # 速度控制(度每秒)
# 示例:
ms4010_multi motor 1 speed 1000  # 以1000度/秒运行
ms4010_multi motor 1 speed -500  # 反向500度/秒运行
ms4010_multi motor 1 speed 0     # 停止
```

### 测试命令

#### 功能测试
```bash
ms4010_multi test_basic        # 基础通信测试
ms4010_multi test_sync         # 同步控制测试
ms4010_multi test_performance  # 性能测试
```

#### 自动测试
```bash
ms4010_multi start_auto        # 启动自动测试线程
ms4010_multi stop_auto         # 停止自动测试线程
```

## 使用示例

### 示例1: 双电机同步控制
```bash
# 1. 初始化系统
ms4010_multi init

# 2. 添加两个电机
ms4010_multi add 1
ms4010_multi add 2

# 3. 开启电机
ms4010_multi motor 1 on
ms4010_multi motor 2 on

# 4. 同步位置控制
ms4010_multi motor 1 pos 9000    # 1号电机到90度
ms4010_multi motor 2 pos -9000   # 2号电机到-90度

# 5. 检查状态
ms4010_multi status

# 6. 关闭电机
ms4010_multi motor 1 off
ms4010_multi motor 2 off
```

### 示例2: 多电机编队控制
```bash
# 1. 系统初始化
ms4010_multi init

# 2. 添加4个电机
ms4010_multi add 1
ms4010_multi add 2
ms4010_multi add 3
ms4010_multi add 4

# 3. 批量开启
for i in 1 2 3 4; do ms4010_multi motor $i on; done

# 4. 编队运动
ms4010_multi motor 1 pos 0
ms4010_multi motor 2 pos 9000
ms4010_multi motor 3 pos 18000
ms4010_multi motor 4 pos 27000

# 5. 运行性能测试
ms4010_multi test_performance

# 6. 启动自动测试
ms4010_multi start_auto
```

### 示例3: 生产线测试场景
```bash
# 1. 快速系统检测
ms4010_multi init
ms4010_multi add 1; ms4010_multi add 2; ms4010_multi add 3

# 2. 基础功能验证
ms4010_multi test_basic

# 3. 同步性能测试
ms4010_multi test_sync

# 4. 通信性能评估
ms4010_multi test_performance

# 5. 获取测试报告
ms4010_multi status
```

## 编程接口

### C语言接口

#### 管理器初始化
```c
ms4010_manager_t manager;
rt_err_t result = ms4010_manager_init(&manager, "uart2", 115200);
```

#### 添加电机
```c
ms4010_device_t motor1;
rt_err_t result = ms4010_manager_add_motor(&manager, &motor1, 1);
```

#### 获取电机设备
```c
ms4010_device_t *motor = ms4010_manager_get_motor(&manager, 1);
if (motor != RT_NULL) {
    ms4010_position_control(motor, 9000, 2000);
}
```

#### 状态查询
```c
rt_err_t result = ms4010_manager_get_all_status(&manager);
```

## 技术规格

### 系统参数
- **最大电机数量**: 16个
- **电机ID范围**: 1-255
- **RS485波特率**: 115200 (可配置)
- **总线仲裁**: 互斥锁机制
- **响应超时**: 1000ms

### 性能指标
- **单命令延时**: ~10-50ms (取决于命令类型)
- **多电机轮询**: ~100-200ms (16个电机)
- **通信成功率**: >99% (正常环境)

### 兼容性
- **向下兼容**: 完全兼容原有单电机接口
- **混合使用**: 可以同时使用管理器和独立电机
- **线程安全**: 支持多线程并发访问

## 故障排除

### 常见问题

#### 1. 电机添加失败
- 检查电机ID是否重复
- 确认管理器已正确初始化
- 验证电机ID在有效范围内(1-255)

#### 2. 通信超时
- 检查RS485总线连接
- 确认电机ID设置正确
- 验证波特率匹配

#### 3. 性能问题
- 适当设置命令间延时
- 避免过度频繁的状态查询
- 使用性能测试评估系统负载

### 调试建议
1. 使用`ms4010_multi status`查看系统状态
2. 运行`ms4010_multi test_basic`验证基础功能
3. 通过性能测试评估系统能力
4. 查看错误计数识别问题电机

## 最佳实践

### 1. 系统设计
- 合理规划电机ID分配
- 预留足够的命令间隔时间
- 实施适当的错误处理机制

### 2. 控制策略
- 批量操作时适当延时
- 优先处理关键电机
- 定期检查系统状态

### 3. 维护建议
- 定期运行性能测试
- 监控错误计数变化
- 及时处理通信异常

这个多电机控制系统为您提供了强大、灵活、可靠的MS4010电机集群控制能力！
