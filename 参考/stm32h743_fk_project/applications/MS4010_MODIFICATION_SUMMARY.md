# MS4010驱动测试文件修改总结

## 修改的文件列表

1. **ms4010_test.c** - 完整测试文件
2. **ms4010_simple_test.c** - 简单测试文件
3. **ms4010_sample.c** - 示例文件
4. **MS4010_TEST_USAGE.md** - 使用说明文档

## 主要修改内容

### 1. 数据结构适配

**原始结构使用问题：**
- 测试文件错误地从 `ms4010_status1_t` 中读取 `speed` 和 `encoder` 字段
- 错误地使用了 `status1.iq` 字段名称
- 错误地从 `ms4010_status3_t` 中读取不存在的字段

**修改后的正确使用：**
- `ms4010_status1_t`: 包含 `temperature`, `voltage`, `current`, `motor_state`, `error_state`
- `ms4010_status_t` (状态2): 包含 `temperature`, `power_or_torque`, `speed`, `encoder`
- `ms4010_status3_t`: 包含 `temperature`, `current_a`, `current_b`, `current_c`

### 2. 函数接口更新

**位置控制函数：**
- `ms4010_position_control()` → `ms4010_position_control2()`
- 速度参数单位从 dps 改为 0.01dps (需要乘以100)

**新增函数支持：**
- `ms4010_single_position_control2()` - 单圈位置控制
- `ms4010_increment_position_control2()` - 增量位置控制
- `ms4010_motor_on()` / `ms4010_motor_off()` - 电机开关控制
- `ms4010_clear_error()` - 错误清除

### 3. 测试命令扩展

**ms4010_test 新增命令：**
- `single <dir> <angle>` - 单圈位置控制测试
- `increment <angle>` - 增量位置控制测试
- `torque <value>` - 力矩控制测试
- `motor_on` / `motor_off` - 电机开关测试
- `brake <cmd>` - 抱闸控制测试
- `clear_error` - 错误清除测试

**ms4010_simple 新增命令：**
- `status2` - 读取状态2
- `motor_on` / `motor_off` - 电机开关
- `clear_err` - 错误清除

**ms4010_test (示例文件) 更新命令：**
- `status2` - 读取状态2
- `single <dir> <angle>` - 单圈位置控制
- `increment <angle>` - 增量位置控制
- 修正了位置控制参数单位

### 4. 状态显示优化

**分离状态显示：**
- 状态1: 显示电压、电流、电机状态、错误状态
- 状态2: 显示速度、编码器、功率/力矩值
- 状态3: 显示三相电流值

**单位转换修正：**
- 电压: `voltage * 0.01f` (0.01V/LSB)
- 电流: `current * 0.01f` (0.01A/LSB)
- 位置控制速度: 输入值为 0.01dps 单位

## 使用示例

### 基本测试流程
```bash
# 1. 初始化设备
ms4010_simple init

# 2. 读取所有状态
ms4010_simple status1    # 电压电流状态
ms4010_simple status2    # 速度编码器状态  
ms4010_simple status3    # 三相电流状态

# 3. 电机控制测试
ms4010_simple motor_on   # 开启电机
ms4010_simple forward    # 正转测试
ms4010_simple status2    # 查看速度
ms4010_simple stop       # 停止电机
```

### 高级控制测试
```bash
# 1. 完整测试
ms4010_test init
ms4010_test control      # 运行控制功能测试

# 2. 位置控制测试
ms4010_test pos 9000     # 移动到90度 (90*100)
ms4010_test single 0 18000  # 顺时针转到180度
ms4010_test increment 4500  # 增量45度

# 3. 示例文件测试
ms4010_test open 300     # 开环控制
ms4010_test speed 360    # 360dps速度控制
ms4010_test pos 18000 36000  # 180度位置，360dps最大速度
```

## 注意事项

1. **参数单位**：
   - 角度参数：0.01度单位 (例如90度输入9000)
   - 位置控制速度：0.01dps单位 (例如360dps输入36000)
   - 普通速度控制：dps单位

2. **状态读取**：
   - 使用正确的状态结构体读取对应信息
   - 状态1用于电压电流监控
   - 状态2用于速度位置监控
   - 状态3用于三相电流监控

3. **错误处理**：
   - 出现通信错误时检查RS485连接
   - 使用 `clear_error` 命令清除电机错误标志
   - 检查电机供电和ID配置

## 文件兼容性

所有修改后的文件都与新的 `drv_ms4010.h` 驱动头文件完全兼容，可以正确编译和运行。测试文件提供了全面的功能验证，可以有效测试MS4010电机驱动的各项功能。
