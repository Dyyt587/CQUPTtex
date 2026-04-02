/**
 * @file laser_gimbal_example.c
 * @brief 激光云台快速使用示例
 * @version 1.0
 * @date 2025-07-28
 * @author Your Name
 * @copyright Copyright (c) 2025
 */

#include "laser_gimbal.h"
#include "drv_ms4010.h"
#include <rtthread.h>

/* 快速使用示例 */

/**
 * @brief 激光云台快速演示
 * 
 * 这个示例展示了如何快速设置和使用激光云台系统
 */
void laser_gimbal_quick_demo(void)
{
    rt_err_t result;
    laser_gimbal_t gimbal;
    ms4010_manager_t motor_manager;
    ms4010_device_t *yaw_motor, *pitch_motor;

    LOG_D("=== 激光云台快速演示 ===\n");

    /* 步骤1: 初始化电机管理器 */
    LOG_D("1. 初始化电机系统...\n");
    result = ms4010_manager_init(&motor_manager, "uart2", 115200);
    if (result != RT_EOK)
    {
        LOG_D("❌ 电机管理器初始化失败: %d\n", result);
        return;
    }

    /* 步骤2: 自动扫描电机 */
    LOG_D("2. 扫描电机设备...\n");
    rt_uint8_t found_count = ms4010_manager_quick_scan(&motor_manager);
    if (found_count < 2)
    {
        LOG_D("❌ 需要至少2个电机，实际发现: %d\n", found_count);
        ms4010_manager_deinit(&motor_manager);
        return;
    }
    LOG_D("✅ 发现 %d 个电机\n", found_count);

    /* 步骤3: 获取电机设备 */
    yaw_motor = ms4010_manager_get_motor(&motor_manager, 1);    // 偏航轴 (水平)
    pitch_motor = ms4010_manager_get_motor(&motor_manager, 2);  // 俯仰轴 (垂直)

    if (yaw_motor == RT_NULL || pitch_motor == RT_NULL)
    {
        LOG_D("❌ 无法获取电机设备 (需要ID=1和ID=2)\n");
        ms4010_manager_deinit(&motor_manager);
        return;
    }
    LOG_D("✅ 电机设备准备就绪\n");

    /* 步骤4: 初始化激光云台 */
    LOG_D("3. 初始化激光云台...\n");
    result = laser_gimbal_init(&gimbal, yaw_motor, pitch_motor, 1.5f); // 1.5米高度
    if (result != RT_EOK)
    {
        LOG_D("❌ 激光云台初始化失败: %d\n", result);
        ms4010_manager_deinit(&motor_manager);
        return;
    }
    LOG_D("✅ 激光云台初始化成功\n");

    /* 步骤5: 配置参数 */
    LOG_D("4. 配置参数...\n");
    
    // 设置运动参数
    laser_gimbal_set_motion_params(&gimbal, 30.0f, 25.0f, 60.0f, 0.8f);
    
    // 设置角度限制
    laser_gimbal_set_angle_limits(&gimbal, -90.0f, 90.0f, -15.0f, 45.0f);
    
    LOG_D("✅ 参数配置完成\n");

    /* 步骤6: 启用电机 */
    LOG_D("5. 启用电机...\n");
    result = laser_gimbal_enable(&gimbal, true);
    if (result != RT_EOK)
    {
        LOG_D("❌ 电机启用失败: %d\n", result);
        goto cleanup;
    }
    LOG_D("✅ 电机已启用\n");

    /* 步骤7: 回零操作 */
    LOG_D("6. 云台回零...\n");
    laser_gimbal_home(&gimbal);
    rt_thread_mdelay(2000);
    LOG_D("✅ 回零完成\n");

    /* 步骤8: 演示各种控制方式 */
    LOG_D("\n=== 开始演示控制功能 ===\n");

    // 演示1: 坐标控制
    LOG_D("\n📍 演示1: 坐标控制\n");
    LOG_D("   指向: 距离5米, 前方4米, 右侧2米\n");
    result = laser_gimbal_point_to_coordinate(&gimbal, 5.0f, 4.0f, 2.0f);
    if (result == RT_EOK)
    {
        LOG_D("   ✅ 已指向目标\n");
        rt_thread_mdelay(3000);
    }

    // 演示2: 角度控制
    LOG_D("\n📐 演示2: 角度控制\n");
    LOG_D("   设置角度: 偏航30°, 俯仰20°\n");
    result = laser_gimbal_set_angle(&gimbal, 30.0f, 20.0f);
    if (result == RT_EOK)
    {
        LOG_D("   ✅ 角度设置完成\n");
        rt_thread_mdelay(2000);
    }

    // 演示3: 多点指向
    LOG_D("\n🎯 演示3: 多点指向\n");
    
    struct {
        float distance, x, y;
        const char *desc;
    } demo_points[] = {
        {6.0f, 6.0f, 0.0f, "正前方6米"},
        {8.0f, 5.0f, 5.0f, "右前方8米"},
        {7.0f, 0.0f, 7.0f, "右侧7米"},
        {5.0f, -3.0f, 3.0f, "左前方5米"}
    };

    for (int i = 0; i < 4; i++)
    {
        LOG_D("   → %s\n", demo_points[i].desc);
        laser_gimbal_point_to_coordinate(&gimbal, 
            demo_points[i].distance, 
            demo_points[i].x, 
            demo_points[i].y);
        rt_thread_mdelay(2000);
    }

    // 演示4: 扫射模式
    LOG_D("\n🔄 演示4: 水平扫射\n");
    for (int angle = -45; angle <= 45; angle += 15)
    {
        LOG_D("   扫射角度: %d°\n", angle);
        laser_gimbal_set_angle(&gimbal, (float)angle, 10.0f);
        rt_thread_mdelay(800);
    }

    // 演示完成，回零
    LOG_D("\n🏠 演示完成，回零\n");
    laser_gimbal_home(&gimbal);
    rt_thread_mdelay(2000);

    LOG_D("\n=== 演示完成 ===\n");
    LOG_D("💡 提示: 你现在可以使用以下命令控制激光云台:\n");
    LOG_D("   laser_point <距离> <x> <y>  - 坐标控制\n");
    LOG_D("   laser_angle <偏航> <俯仰>   - 角度控制\n");
    LOG_D("   laser_home                  - 回零\n");
    LOG_D("   laser_status                - 查看状态\n");

cleanup:
    /* 清理资源 */
    laser_gimbal_enable(&gimbal, false);
    laser_gimbal_deinit(&gimbal);
    ms4010_manager_deinit(&motor_manager);
}

/**
 * @brief 简单的坐标控制示例
 */
void simple_coordinate_control_example(void)
{
    // 假设云台已经初始化 (可以先运行laser_init命令)
    
    LOG_D("=== 简单坐标控制示例 ===\n");
    
    // 创建一个全局云台对象的外部引用
    extern laser_gimbal_t g_laser_gimbal;
    
    /* 示例1: 指向建筑物 */
    LOG_D("指向前方大楼 (距离20米, 前方18米, 右侧8米)\n");
    laser_gimbal_point_to_coordinate(&g_laser_gimbal, 20.0f, 18.0f, 8.0f);
    rt_thread_mdelay(3000);
    
    /* 示例2: 指向路标 */
    LOG_D("指向路标 (距离15米, 前方12米, 左侧9米)\n");
    laser_gimbal_point_to_coordinate(&g_laser_gimbal, 15.0f, 12.0f, -9.0f);
    rt_thread_mdelay(3000);
    
    /* 示例3: 指向天空 */
    LOG_D("指向天空 (距离30米, 前方25米, 高度15米)\n");
    laser_coordinate_t sky_target = {25.0f, 0.0f, 15.0f};
    laser_gimbal_point_to_3d_coordinate(&g_laser_gimbal, sky_target);
    rt_thread_mdelay(3000);
    
    /* 回零 */
    LOG_D("回到初始位置\n");
    laser_gimbal_home(&g_laser_gimbal);
    
    LOG_D("示例完成！\n");
}

/**
 * @brief 实际应用场景示例 - 安防巡逻
 */
void security_patrol_example(void)
{
    extern laser_gimbal_t g_laser_gimbal;
    
    LOG_D("=== 安防巡逻模式 ===\n");
    
    /* 定义巡逻点 */
    struct patrol_point {
        float distance, x, y;
        const char *location;
        uint32_t dwell_time_ms;  // 停留时间
    } patrol_points[] = {
        {25.0f, 25.0f, 0.0f, "大门入口", 2000},
        {20.0f, 15.0f, 12.0f, "停车区域", 1500},
        {30.0f, 0.0f, 30.0f, "侧面围墙", 2000},
        {18.0f, -12.0f, 12.0f, "后门区域", 1500},
        {22.0f, -20.0f, -8.0f, "垃圾区域", 1000},
        {15.0f, 10.0f, -10.0f, "办公区域", 1000}
    };
    
    size_t point_count = sizeof(patrol_points) / sizeof(patrol_points[0]);
    
    LOG_D("开始巡逻，共 %d 个检查点\n", point_count);
    
    for (int cycle = 0; cycle < 2; cycle++)  // 巡逻2轮
    {
        LOG_D("\n--- 第 %d 轮巡逻 ---\n", cycle + 1);
        
        for (size_t i = 0; i < point_count; i++)
        {
            LOG_D("检查点 %d: %s\n", i + 1, patrol_points[i].location);
            
            rt_err_t result = laser_gimbal_point_to_coordinate(&g_laser_gimbal,
                patrol_points[i].distance,
                patrol_points[i].x,
                patrol_points[i].y);
                
            if (result == RT_EOK)
            {
                LOG_D("  ✅ 已指向 - 监控中...\n");
                rt_thread_mdelay(patrol_points[i].dwell_time_ms);
            }
            else
            {
                LOG_D("  ❌ 指向失败\n");
            }
        }
    }
    
    LOG_D("\n巡逻完成，回到待机位置\n");
    laser_gimbal_home(&g_laser_gimbal);
}

/* MSH命令导出 */
MSH_CMD_EXPORT(laser_gimbal_quick_demo, Quick demo of laser gimbal system);
MSH_CMD_EXPORT(simple_coordinate_control_example, Simple coordinate control example);
MSH_CMD_EXPORT(security_patrol_example, Security patrol application example);

/**
 * @brief 校准功能演示 (新的自动保存模式)
 */
void calibration_demo_example(void)
{
    extern laser_gimbal_t g_laser_gimbal;
    
    LOG_D("=== 激光云台自动校准演示 ===\n");
    LOG_D("⚠️  注意: 这是新的自动校准模式演示！\n\n");
    
    /* 1. 配置监控参数 */
    LOG_D("1. 配置校准监控参数...\n");
    laser_gimbal_set_calibration_monitor_params(&g_laser_gimbal, 0.3f, 2000, true);
    rt_thread_mdelay(1000);
    
    /* 2. 进入校准模式 */
    LOG_D("2. 进入校准模式 (电机将关闭)...\n");
    laser_gimbal_calibration_mode(&g_laser_gimbal, true);
    rt_thread_mdelay(2000);
    
    LOG_D("\n🖐️  现在电机已关闭，您可以:\n");
    LOG_D("   1. 用手调整云台指向目标中心\n");
    LOG_D("   2. 保持位置稳定2秒钟\n");
    LOG_D("   3. 系统将自动计算偏移并保存\n");
    LOG_D("   4. 观察控制台的实时角度显示\n\n");
    
    /* 3. 模拟等待用户手动调整 */
    LOG_D("3. 等待手动调整 (演示模式下等待10秒)...\n");
    for (int i = 10; i > 0; i--)
    {
        LOG_D("   倒计时: %d秒 (实际使用时请手动调整云台)\n", i);
        rt_thread_mdelay(1000);
    }
    
    /* 4. 演示查看当前角度 */
    LOG_D("4. 查看当前角度信息:\n");
    float yaw_raw, pitch_raw;
    laser_gimbal_get_motor_raw_angle(&g_laser_gimbal, &yaw_raw, &pitch_raw);
    LOG_D("   当前角度: 偏航=%.2f°, 俯仰=%.2f°\n", yaw_raw, pitch_raw);
    LOG_D("   偏移设置: 偏航=%.2f°, 俯仰=%.2f°\n", 
              g_laser_gimbal.yaw_offset, g_laser_gimbal.pitch_offset);
    rt_thread_mdelay(2000);
    
    /* 5. 等待一段时间让监控线程工作 */
    LOG_D("5. 监控角度稳定性 (观察实时显示)...\n");
    rt_thread_mdelay(5000);
    
    /* 6. 退出校准模式 */
    LOG_D("6. 退出校准模式 (电机将重新启用)...\n");
    laser_gimbal_calibration_mode(&g_laser_gimbal, false);
    rt_thread_mdelay(2000);
    
    /* 7. 测试校准结果 */
    LOG_D("7. 测试校准结果:\n");
    LOG_D("   指向 0°, 0° (应该指向目标中心)...\n");
    laser_gimbal_set_angle(&g_laser_gimbal, 0.0f, 0.0f);
    rt_thread_mdelay(3000);
    
    LOG_D("   指向 30°, 15° (应该指向右上方)...\n");
    laser_gimbal_set_angle(&g_laser_gimbal, 30.0f, 15.0f);
    rt_thread_mdelay(3000);
    
    LOG_D("   回到初始位置...\n");
    laser_gimbal_home(&g_laser_gimbal);
    
    LOG_D("\n=== 自动校准演示完成 ===\n");
    LOG_D("💡 实际使用时的简化流程:\n");
    LOG_D("   1. laser_calibration on     # 进入校准模式\n");
    LOG_D("   2. 手动调整云台指向目标     # 用手调整\n");
    LOG_D("   3. 保持稳定等待自动保存     # 等待2-3秒\n");
    LOG_D("   4. laser_calibration off    # 退出校准模式\n");
    LOG_D("   5. 测试: laser_angle 0 0    # 验证校准结果\n\n");
    LOG_D("🔧 可选配置命令:\n");
    LOG_D("   laser_cal_config 0.5 3.0 1  # 配置监控参数\n");
    LOG_D("   laser_cal_show              # 查看当前状态\n");
}
MSH_CMD_EXPORT(calibration_demo_example, Calibration functionality demonstration);
