# MS4010驱动快速使用指南

## 快速开始（5分钟上手）

### 1. 确认硬件连接
- MS4010电机通过RS485连接到开发板
- 确认RS485的A、B线连接正确
- 确认电源和地线连接

### 2. 配置RS485设备
在你的项目中确保RS485设备已配置，通常在以下文件中：
```c
// packages/rs485-latest/src/rs485_dev_cfg.h
#define RS485_DEV_CFG_TABLE \
{ \
    {"uart2", 115200, 0, -1, 0, "rs485_0"}, \
}
```

### 3. 编译并下载程序
确保以下文件已添加到工程：
- `drv_ms4010.h`
- `drv_ms4010.c`  
- `ms4010_simple_test.c` (用于测试)

### 4. 基本测试步骤

#### 方法1：使用MSH命令（推荐）
在终端中输入以下命令：

```bash
# 1. 初始化电机
ms4010_simple init

# 2. 测试正转
ms4010_simple forward

# 3. 查看状态
ms4010_simple status

# 4. 停止电机
ms4010_simple stop

# 5. 测试反转
ms4010_simple reverse

# 6. 停止电机
ms4010_simple stop
```

#### 方法2：自动循环测试
取消`ms4010_simple_test.c`文件最后一行的注释：
```c
INIT_APP_EXPORT(ms4010_simple_test_start);
```
重新编译下载后，程序会自动开始循环测试。

### 5. 预期结果
如果一切正常，你应该看到类似以下的日志输出：
```
[I/ms4010.simple] MS4010 initialized successfully!
[I/ms4010.simple] === Test Loop 1 ===
[I/ms4010.simple] Step 1: Forward rotation (200)
[I/ms4010.simple]   Command sent successfully
[I/ms4010.simple]   Motor status: Temp=25°C, Speed=180 rpm, Encoder=1234
```

## 常见问题排查

### 问题1：初始化失败
**现象**：`Failed to initialize MS4010: -1`

**排查步骤**：
1. 检查RS485设备名称是否正确（通常是"uart2"或"uart1"）
2. 确认RS485硬件连接
3. 检查电机是否上电

**解决方案**：
修改`ms4010_simple_test.c`中的设备名称：
```c
#define SIMPLE_TEST_RS485_NAME "uart1"  // 改为你实际使用的串口
```

### 问题2：通信超时
**现象**：`Expected 13 bytes, received 0 bytes`

**排查步骤**：
1. 检查RS485 A、B线是否接反
2. 确认波特率设置（默认115200）
3. 检查电机ID是否正确（默认1）
4. 检查电机是否正确上电

### 问题3：校验和错误
**现象**：`Header checksum error` 或 `Data checksum error`

**排查步骤**：
1. 检查RS485总线是否有干扰
2. 降低波特率尝试
3. 检查总线终端电阻

## 高级使用

### 使用编程接口
```c
#include "drv_ms4010.h"

ms4010_device_t motor;

// 初始化
ms4010_init(&motor, "uart2", 1, 115200);

// 控制电机
ms4010_speed_control(&motor, 360);      // 360度/秒
ms4010_position_control(&motor, 18000, 180); // 转到180度

// 读取状态
ms4010_status_t status;
ms4010_get_status(&motor, &status);
```

### 完整测试程序
如果需要更完整的测试，使用`ms4010_test.c`：
```bash
ms4010_test init     # 初始化
ms4010_test basic    # 基础测试
ms4010_test control  # 控制功能测试
ms4010_test start    # 启动连续测试
ms4010_test stop     # 停止连续测试
```

## 参数说明

### 电机ID
- 范围：1-255
- 必须与电机硬件设置一致

### 控制值范围
- 开环控制：-1000 ~ 1000
- 速度控制：度/秒 (dps)
- 位置控制：0.01度为单位

### 响应数据
- 温度：摄氏度
- 功率/力矩：具体单位取决于电机型号
- 速度：RPM
- 编码器：编码器计数值

## 注意事项

1. **线程安全**：驱动已经实现线程安全，可以在多线程环境中使用
2. **超时设置**：默认通信超时1秒，可根据需要调整
3. **错误处理**：所有API都会返回错误码，请检查返回值
4. **电机安全**：测试时注意电机运行安全，确保有足够空间

## 下一步

完成基本测试后，你可以：
1. 根据实际需求修改控制参数
2. 集成到你的应用代码中
3. 添加更多的错误处理逻辑
4. 实现更复杂的运动控制算法
