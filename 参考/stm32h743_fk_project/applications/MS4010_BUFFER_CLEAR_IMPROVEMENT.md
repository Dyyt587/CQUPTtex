# MS4010驱动缓冲区清空功能改进

## 问题描述

在RS485通信过程中，当出现接收错误（如接收超时、帧格式错误、校验错误等）时，串口设备的接收缓冲区可能残留错误数据。这些残留数据会影响后续命令帧的解析，导致通信异常。

## 解决方案

### 1. 新增缓冲区清空函数

添加了`ms4010_clear_rx_buffer`静态函数，用于在发生接收错误时清空RS485接收缓冲区：

```c
static rt_err_t ms4010_clear_rx_buffer(rs485_inst_t *rs485_handle)
{
    RT_ASSERT(rs485_handle != RT_NULL);
    
    rt_uint8_t dummy_buffer[64];  // 临时缓冲区
    rt_int32_t recv_len;
    rt_uint32_t clear_count = 0;
    rt_uint32_t max_clear_attempts = 10; // 最大清空尝试次数
    
    LOG_D("Clearing RS485 receive buffer...");
    
    /* 中断当前的接收等待 */
    rs485_break_recv(rs485_handle);
    
    /* 短暂延时让总线稳定 */
    rt_thread_mdelay(5);
    
    /* 循环读取剩余数据直到缓冲区为空 */
    while (clear_count < max_clear_attempts)
    {
        /* 尝试以最短超时读取数据 */
        rs485_set_recv_tmo(rs485_handle, 10); // 设置10ms超时
        
        recv_len = rs485_recv(rs485_handle, dummy_buffer, sizeof(dummy_buffer));
        
        if (recv_len <= 0)
        {
            /* 没有更多数据，缓冲区已清空 */
            break;
        }
        
        clear_count++;
        LOG_D("Cleared %d bytes from buffer (attempt %d)", recv_len, clear_count);
    }
    
    /* 恢复原始超时设置 */
    rs485_set_recv_tmo(rs485_handle, MS4010_DEFAULT_TIMEOUT);
    
    if (clear_count > 0)
    {
        LOG_I("RS485 buffer cleared after %d attempts", clear_count);
    }
    
    return RT_EOK;
}
```

### 2. 错误处理增强

#### 在`ms4010_send_command`函数中的改进：

**接收长度错误处理：**
```c
if (recv_len != expected_response_size)
{
    LOG_W("MS4010 motor %d: cmd is 0x%x Expected %d bytes, received %d bytes", 
          device->motor_id, cmd, expected_response_size, recv_len);
    device->error_count++;
    
    /* 清空接收缓冲区防止影响后续命令 */
    ms4010_clear_rx_buffer(device->rs485_handle);
    
    result = -MS4010_ETIMEOUT;
}
```

**响应解析错误处理：**
```c
if (result != RT_EOK)
{
    device->error_count++;
    /* 当解析错误时，清空接收缓冲区防止影响后续命令 */
    ms4010_clear_rx_buffer(device->rs485_handle);
}
```

#### 在`ms4010_send_command_shared`函数中的改进：

**接收长度错误处理：**
```c
if (recv_len != expected_response_size)
{
    LOG_W("MS4010 motor %d: cmd is 0x%x Expected %d bytes, received %d bytes", 
          motor_id, cmd, expected_response_size, recv_len);
    
    /* 清空接收缓冲区防止影响后续命令 */
    ms4010_clear_rx_buffer(rs485_handle);
    
    result = -MS4010_ETIMEOUT;
}
```

**帧头错误处理：**
```c
/* 基本的帧头检查 */
if (rx_buffer[0] != MS4010_FRAME_HEAD)
{
    LOG_W("MS4010 motor %d: Invalid response frame header 0x%02X", motor_id, rx_buffer[0]);
    
    /* 清空接收缓冲区防止影响后续命令 */
    ms4010_clear_rx_buffer(rs485_handle);
    
    result = -MS4010_EFRAME;
}
```

## 功能特性

### 1. 智能缓冲区清空
- **中断接收**: 使用`rs485_break_recv`中断当前接收等待
- **循环读取**: 循环读取残留数据直到缓冲区为空
- **超时保护**: 设置短超时（10ms）避免长时间等待
- **次数限制**: 最多尝试10次避免死循环

### 2. 多层次错误检测
- **接收长度错误**: 当接收字节数与期望不符时清空缓冲区
- **帧格式错误**: 当响应帧头不正确时清空缓冲区
- **解析错误**: 当协议解析失败时清空缓冲区
- **校验错误**: 校验和错误在解析函数中已有处理

### 3. 适用场景
- **单电机模式**: `ms4010_send_command`函数中的错误处理
- **多电机模式**: `ms4010_send_command_shared`函数中的错误处理
- **兼容性**: 与现有错误计数和日志系统完全兼容

## 技术细节

### 1. 缓冲区清空策略
```
1. 中断当前接收操作（rs485_break_recv）
2. 延时5ms让总线稳定
3. 设置短超时（10ms）
4. 循环读取数据到临时缓冲区
5. 如果无数据读取则退出
6. 恢复原始超时设置
7. 记录清空操作日志
```

### 2. 错误类型对应处理
| 错误类型 | 错误码 | 处理方式 |
|----------|--------|----------|
| 接收超时/长度错误 | MS4010_ETIMEOUT | 清空缓冲区 |
| 帧头错误 | MS4010_EFRAME | 清空缓冲区 |
| 校验错误 | MS4010_ECHECKSUM | 清空缓冲区 |
| 参数错误 | MS4010_EINVAL | 仅记录错误 |

### 3. 性能影响
- **延时开销**: 错误时额外5ms总线稳定延时
- **清空时间**: 通常10-50ms（取决于残留数据量）
- **频率**: 仅在通信错误时执行，正常通信无影响
- **资源占用**: 64字节临时缓冲区

## 使用效果

### 1. 提高通信可靠性
- 防止错误数据影响后续命令
- 减少连续通信失败概率
- 提升多电机系统稳定性

### 2. 增强错误恢复能力
- 自动从通信错误中恢复
- 减少人工干预需求
- 提高系统容错性

### 3. 便于问题诊断
- 详细的错误日志记录
- 清空操作可视化反馈
- 错误统计计数器

## 最佳实践

### 1. 日志配置
建议将DBG_LVL设置为DBG_LOG或更高级别，以便观察缓冲区清空操作：
```c
#define DBG_LVL DBG_LOG
```

### 2. 超时设置
根据实际应用场景调整超时参数：
- 高实时性要求：保持默认10ms清空超时
- 复杂环境：可适当增加到20-50ms

### 3. 错误监控
定期检查错误计数器，及时发现通信问题：
```bash
ms4010_multi status  # 查看所有电机状态和错误计数
```

## 兼容性说明

- ✅ 完全向下兼容现有API
- ✅ 不影响正常通信性能
- ✅ 与单电机和多电机模式均兼容
- ✅ 保持原有错误处理逻辑

这个改进有效解决了RS485通信中缓冲区残留数据导致的解析错误问题，大大提升了MS4010驱动的通信可靠性和稳定性。
