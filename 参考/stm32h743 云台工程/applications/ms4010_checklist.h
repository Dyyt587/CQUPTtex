/**
 * @file ms4010_checklist.h
 * @brief MS4010驱动配置检查清单
 * @version 1.0
 * @date 2025-07-25
 */

#ifndef __MS4010_CHECKLIST_H__
#define __MS4010_CHECKLIST_H__

/*
 * MS4010驱动配置检查清单
 * 使用前请确认以下配置项：
 */

/* ====== 1. RT-Thread配置检查 ====== */
#ifndef RT_USING_DEVICE
#error "请在rtconfig.h中启用 RT_USING_DEVICE"
#endif

#ifndef RT_USING_SERIAL
#error "请在rtconfig.h中启用 RT_USING_SERIAL"
#endif

/* ====== 2. RS485包配置检查 ====== */
#ifndef PKG_USING_RS485
#error "请在menuconfig中启用RS485软件包：RT-Thread online packages -> peripheral libraries -> rs485"
#endif

/* ====== 3. 串口设备检查 ====== */
/*
 * 请确认以下串口设备名称与你的硬件配置一致
 * 常见的串口设备名称：
 * - "uart1" 
 * - "uart2"
 * - "uart3"
 * 等等
 */
#warning "请检查MS4010_DEFAULT_RS485_NAME是否与实际硬件匹配"

/* ====== 4. 硬件连接检查清单 ====== */
/*
 * [ ] MS4010电机电源连接正确
 * [ ] RS485 A线连接正确  
 * [ ] RS485 B线连接正确
 * [ ] 地线连接正确
 * [ ] RS485总线终端电阻配置正确（如果需要）
 */

/* ====== 5. 软件配置检查清单 ====== */
/*
 * [ ] RS485设备已在rs485_dev_cfg.h中正确配置
 * [ ] 电机ID与硬件设置一致（默认1）
 * [ ] 波特率与电机设置一致（默认115200）
 * [ ] 串口设备名称正确
 */

/* ====== 6. 编译配置检查 ====== */
/*
 * 请确保以下文件已添加到工程：
 * [ ] applications/drv_ms4010.h
 * [ ] applications/drv_ms4010.c
 * [ ] applications/ms4010_simple_test.c (可选，用于测试)
 * [ ] applications/ms4010_test.c (可选，用于完整测试)
 */

/* ====== 7. 运行时检查 ====== */
/*
 * 程序运行时检查：
 * [ ] RS485设备能够正常找到和打开
 * [ ] 电机响应正常（无通信超时）
 * [ ] 电机状态数据正确（温度、速度等合理）
 * [ ] 无校验和错误
 */

/* ====== 示例配置 ====== */
/*
// 1. rtconfig.h 中需要的配置示例
#define RT_USING_DEVICE
#define RT_USING_SERIAL
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart3"

// 2. packages/rs485-latest/src/rs485_dev_cfg.h 配置示例
#define RS485_DEV_CFG_TABLE \
{ \
    {"uart2", 115200, 0, -1, 0, "rs485_0"}, \
}

// 3. 在main.c或应用代码中的使用示例
#include "drv_ms4010.h"

static ms4010_device_t motor;

int main(void)
{
    // 初始化MS4010
    if (ms4010_init(&motor, "uart2", 1, 115200) == RT_EOK)
    {
        rt_kprintf("MS4010 initialized successfully\n");
        
        // 测试电机
        ms4010_open_loop_control(&motor, 200);
        rt_thread_mdelay(2000);
        ms4010_open_loop_control(&motor, 0);
    }
    else
    {
        rt_kprintf("MS4010 initialization failed\n");
    }
    
    return 0;
}
*/

#endif /* __MS4010_CHECKLIST_H__ */
