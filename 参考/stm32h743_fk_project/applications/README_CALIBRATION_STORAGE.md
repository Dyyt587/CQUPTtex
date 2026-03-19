# 激光云台校准参数存储机制说明

## 概述

由于嵌入式系统没有文件系统支持，激光云台的校准参数采用内存变量存储机制，确保校准参数在系统运行期间的持久性。

## 存储机制

### 全局变量存储

校准参数保存在全局静态变量 `g_calibration_storage` 中：

```c
typedef struct {
    bool is_valid;                  // 校准参数是否有效
    float yaw_offset;               // 偏航轴偏移量
    float pitch_offset;             // 俯仰轴偏移量
    bool yaw_reverse;               // 偏航轴反向
    bool pitch_reverse;             // 俯仰轴反向
    float gimbal_height;            // 云台高度
    uint32_t save_time;             // 保存时间戳
} laser_gimbal_calibration_storage_t;
```

### 特点

- **运行时持久性**: 参数在系统运行期间保持有效
- **自动加载**: 系统初始化时自动加载已保存的校准参数
- **实时生效**: 保存后立即在云台控制中生效
- **状态跟踪**: 记录保存时间戳和有效性标志

## 使用方法

### 基本操作

1. **保存校准参数**
   ```
   laser_cal_save
   ```

2. **加载校准参数**
   ```
   laser_cal_load
   ```

3. **查看存储状态**
   ```
   laser_cal_storage
   ```

4. **重置存储**
   ```
   laser_cal_reset confirm
   ```

### 校准流程

1. 进入校准模式
   ```
   laser_calibration on
   ```

2. 手动调整云台到目标位置
   ```
   laser_cal_set 0 0
   ```

3. 计算并保存偏移量
   ```
   laser_cal_offset 0 0
   laser_cal_save
   ```

4. 退出校准模式
   ```
   laser_calibration off
   ```

### 自动校准流程

1. 配置监控参数
   ```
   laser_cal_config 0.5 3000 1
   ```

2. 进入校准模式
   ```
   laser_calibration on
   ```

3. 手动调整云台位置，系统将自动监控角度稳定性并保存

## 系统集成

### 初始化时自动加载

系统初始化时会自动尝试加载校准参数：

```c
/* 自动加载校准参数（如果存在） */
if (laser_gimbal_load_calibration(gimbal, RT_NULL) == RT_EOK)
{
    LOG_I("已自动加载校准参数");
}
else
{
    LOG_I("未找到校准参数，将使用默认参数");
}
```

### 实时监控和自动保存

当启用自动保存时，系统会监控角度稳定性：
- 角度稳定阈值：默认 0.5°
- 稳定时间要求：默认 3000ms
- 满足条件时自动保存校准参数

## 注意事项

1. **电源断电**: 校准参数在系统断电后会丢失，需要重新校准
2. **系统重启**: 重启后需要重新进行校准（或实现参数持久化存储）
3. **存储安全**: 使用 `laser_cal_reset confirm` 命令可以完全清除校准参数
4. **参数验证**: 加载参数前会检查有效性标志

## 扩展建议

对于需要断电保持的场景，可以考虑：

1. **Flash存储**: 将参数保存到Flash的特定区域
2. **EEPROM存储**: 使用外部EEPROM芯片存储参数
3. **配置文件**: 在有文件系统时保存到配置文件
4. **网络同步**: 通过网络从服务器获取校准参数

## 故障排除

### 校准参数无效

```
laser_cal_storage
```
检查存储状态，如果显示无效，需要重新校准。

### 参数未生效

```
laser_cal_verify 0 0 1.0
```
验证校准偏移是否正确应用。

### 清除错误参数

```
laser_cal_reset confirm
```
完全重置校准参数存储。
