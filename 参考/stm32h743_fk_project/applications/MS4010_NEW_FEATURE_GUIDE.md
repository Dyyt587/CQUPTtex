# MS4010新增功能使用指南

## 新增功能概述

成功为MS4010驱动添加了"设置当前位置为任意角度"功能，命令码0x95，支持将当前位置设置为任意角度值（写入RAM）。

## 快速使用

### 命令格式
```bash
# 三个测试文件都支持相同的命令
ms4010_test set_pos <angle>
ms4010_simple set_pos <angle>  
ms4010_sample set_pos <angle>
```

### 参数说明
- `angle`: 角度值，单位为0.01度
- 范围: -2,147,483,648 ~ 2,147,483,647
- 示例: 9000 表示 90.00 度

### 常用示例
```bash
# 重置零点
ms4010_test set_pos 0

# 设置为90度
ms4010_test set_pos 9000

# 设置为-180度  
ms4010_test set_pos -18000

# 设置为3.6圈(1296度)
ms4010_test set_pos 129600
```

## 典型应用

### 零点校准
```bash
ms4010_test init
ms4010_test motor_on
# 将电机移动到期望的零点位置
ms4010_test set_pos 0  # 重新定义零点
ms4010_test status     # 验证设置
```

### 位置偏差补偿
```bash
ms4010_test status     # 查看当前显示位置
# 如果有固定偏差，使用set_pos重新定义当前位置
ms4010_test set_pos 0  # 补偿偏差
```

## 技术细节

### 协议实现
- 命令码: 0x95
- 数据长度: 4字节
- 数据格式: int32_t小端序
- 单位: 0.01度/LSB

### 驱动函数
```c
rt_err_t ms4010_set_current_position(ms4010_device_t *device, rt_int32_t angle);
```

## 注意事项

1. 此设置仅保存在RAM中，断电会丢失
2. 不会移动电机，只重新定义位置值
3. 支持多圈角度设置
4. 设置后立即生效

## 完成状态

✅ 驱动函数实现完成  
✅ 三个测试文件都已集成  
✅ MSH命令支持完成  
✅ 文档和示例完成

现在MS4010驱动支持完整的21个函数，包括这个新增的位置设置功能。
