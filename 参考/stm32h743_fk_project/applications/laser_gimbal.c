/**
 * @file laser_gimbal.c
 * @brief 基于MS4010双电机的二维激光云台控制系统实现
 * @version 1.0
 * @date 2025-07-28
 * @author Your Name
 * @copyright Copyright (c) 2025
 */

#include "laser_gimbal.h"
#include "hipnuc_dec.h"  // 添加陀螺仪数据接口

#define DBG_TAG "laser_gimbal"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


/* 全局校准参数存储 */
typedef struct
{
    bool is_valid;       // 校准参数是否有效
    float yaw_offset;    // 偏航轴偏移量
    float pitch_offset;  // 俯仰轴偏移量
    bool yaw_reverse;    // 偏航轴反向
    bool pitch_reverse;  // 俯仰轴反向
    float gimbal_height; // 云台高度
    uint32_t save_time;  // 保存时间戳
} laser_gimbal_calibration_storage_t;

static laser_gimbal_calibration_storage_t g_calibration_storage = {
    .is_valid = false,
    .yaw_offset = 0.0f,
    .pitch_offset = 0.0f,
    .yaw_reverse = false,
    .pitch_reverse = false,
    .gimbal_height = 1.5f,
    .save_time = 0};

/* 内部函数前向声明 */
static rt_err_t laser_gimbal_move_to_angle(laser_gimbal_t *gimbal, laser_gimbal_angle_t target_angle);
static rt_err_t laser_gimbal_update_current_position(laser_gimbal_t *gimbal);
static float laser_gimbal_normalize_angle(float angle);
static rt_err_t laser_gimbal_validate_coordinate(laser_coordinate_t coordinate);
static void laser_gimbal_gyro_compensation_thread_entry(void *parameter);

/* 陀螺仪补偿线程 */
static rt_thread_t g_gyro_compensation_thread = RT_NULL;
static bool g_gyro_compensation_thread_running = false;
static laser_gimbal_t *g_active_gimbal = RT_NULL;

static rt_err_t laser_gimbal_start_gyro_compensation_thread(laser_gimbal_t *gimbal);
static rt_err_t laser_gimbal_stop_gyro_compensation_thread(void);
/**
 * @brief 初始化激光云台系统
 */
rt_err_t laser_gimbal_init(laser_gimbal_t *gimbal,
                           ms4010_device_t *yaw_motor,
                           ms4010_device_t *pitch_motor,
                           float gimbal_height)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(yaw_motor != RT_NULL);
    RT_ASSERT(pitch_motor != RT_NULL);
    RT_ASSERT(gimbal_height > 0.0f);

    /* 清零结构体 */
    rt_memset(gimbal, 0, sizeof(laser_gimbal_t));

    /* 基本参数初始化 */
    gimbal->mode = LASER_GIMBAL_MODE_COORDINATE;
    gimbal->state = LASER_GIMBAL_STATE_IDLE;
    gimbal->yaw_motor = yaw_motor;
    gimbal->pitch_motor = pitch_motor;
    gimbal->gimbal_height = gimbal_height;

    /* 机械参数默认值 */
    gimbal->yaw_offset = 97.07f;
    gimbal->pitch_offset = 17.76f;
    gimbal->yaw_reverse = false;
    gimbal->pitch_reverse = false;

    /* 校准参数初始化 */
    gimbal->calibration_mode = false;
    gimbal->yaw_motor_current_angle = 0.0f;
    gimbal->pitch_motor_current_angle = 0.0f;

    /* 校准监控参数初始化 */
    gimbal->calibration_monitor_thread = RT_NULL;
    gimbal->calibration_monitor_running = false;
    gimbal->angle_stable_threshold = 0.5f; // 默认0.5度稳定阈值
    gimbal->angle_stable_time_ms = 3000;   // 默认3秒稳定时间
    gimbal->last_angle_change_time = 0;
    gimbal->last_stable_yaw = 0.0f;
    gimbal->last_stable_pitch = 0.0f;
    gimbal->auto_save_enabled = true; // 默认启用自动保存

    /* 运动参数默认值 */
    gimbal->motion_params.max_yaw_speed = 72000.0f;   // 30度/秒
    gimbal->motion_params.max_pitch_speed = 30000.0f; // 30度/秒
    gimbal->motion_params.acceleration = 100.0f;   // 100度/秒²
    gimbal->motion_params.smooth_factor = 0.8f;    // 平滑因子

    /* 速度控制参数初始化 */
    gimbal->velocity_control_enabled = false;
    gimbal->current_yaw_speed = 0.0f;
    gimbal->current_pitch_speed = 0.0f;
    gimbal->velocity_update_time = 0;

    /* 陀螺仪补偿参数初始化 */
    gimbal->gyro_compensation_enabled = false;
    gimbal->gyro_compensation_gain_yaw = 0.5f;    // 默认yaw轴增益
    gimbal->gyro_compensation_gain_pitch = 0.5f;  // 默认pitch轴增益
    gimbal->gyro_offset_yaw = 0.0f;               // yaw轴零偏
    gimbal->gyro_offset_pitch = 0.0f;             // pitch轴零偏
    gimbal->gyro_offset_roll = 0.0f;              // roll轴零偏
    gimbal->last_gyro_yaw = 0.0f;                 // 上次yaw轴数据
    gimbal->last_gyro_pitch = 0.0f;               // 上次pitch轴数据
    gimbal->last_gyro_roll = 0.0f;                // 上次roll轴数据
    gimbal->last_gyro_update_time = 0;            // 数据更新时间
    gimbal->gyro_calibration_mode = false;        // 校准模式
    gimbal->gyro_calibration_count = 0;           // 校准计数
    rt_memset(gimbal->gyro_calibration_samples, 0, sizeof(gimbal->gyro_calibration_samples));

    /* 角度限制默认值 */
    gimbal->angle_limits_min.yaw = LASER_GIMBAL_MIN_YAW_ANGLE;
    gimbal->angle_limits_max.yaw = LASER_GIMBAL_MAX_YAW_ANGLE;
    gimbal->angle_limits_min.pitch = LASER_GIMBAL_MIN_PITCH_ANGLE;
    gimbal->angle_limits_max.pitch = LASER_GIMBAL_MAX_PITCH_ANGLE;

    /* 创建互斥锁 */
    gimbal->gimbal_mutex = rt_mutex_create("laser_gimbal_mutex", RT_IPC_FLAG_FIFO);
    if (gimbal->gimbal_mutex == RT_NULL)
    {
        LOG_E("Failed to create gimbal mutex");
        return -RT_ERROR;
    }

    /* 更新当前位置 */
    laser_gimbal_update_current_position(gimbal);

    gimbal->last_update_time = rt_tick_get();

    /* 自动加载校准参数（如果存在） */
    if (laser_gimbal_load_calibration(gimbal, RT_NULL) == RT_EOK)
    {
        LOG_I("已自动加载校准参数");
    }
    else
    {
        LOG_I("未找到校准参数，将使用默认参数");
    }

    LOG_I("Laser gimbal initialized successfully (height: %.2fm)", gimbal_height);
    return RT_EOK;
}

/**
 * @brief 反初始化激光云台系统
 */
rt_err_t laser_gimbal_deinit(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 停止运动 */
    laser_gimbal_stop(gimbal);

    /* 禁用电机 */
    laser_gimbal_enable(gimbal, false);

    /* 停止校准监控线程 */
    if (gimbal->calibration_mode)
    {
        laser_gimbal_stop_calibration_monitor(gimbal);
    }

    /* 释放互斥锁 */
    if (gimbal->gimbal_mutex != RT_NULL)
    {
        rt_mutex_delete(gimbal->gimbal_mutex);
        gimbal->gimbal_mutex = RT_NULL;
    }

    gimbal->state = LASER_GIMBAL_STATE_IDLE;

    LOG_I("Laser gimbal deinitialized");
    return RT_EOK;
}

/**
 * @brief 设置云台机械参数
 */
rt_err_t laser_gimbal_set_mechanical_params(laser_gimbal_t *gimbal,
                                            float yaw_offset,
                                            float pitch_offset,
                                            bool yaw_reverse,
                                            bool pitch_reverse)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->yaw_offset = yaw_offset;
    gimbal->pitch_offset = pitch_offset;
    gimbal->yaw_reverse = yaw_reverse;
    gimbal->pitch_reverse = pitch_reverse;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Mechanical params set: yaw_offset=%.2f, pitch_offset=%.2f, yaw_rev=%d, pitch_rev=%d",
          yaw_offset, pitch_offset, yaw_reverse, pitch_reverse);
    return RT_EOK;
}

/**
 * @brief 设置云台运动参数
 */
rt_err_t laser_gimbal_set_motion_params(laser_gimbal_t *gimbal,
                                        float max_yaw_speed,
                                        float max_pitch_speed,
                                        float acceleration,
                                        float smooth_factor)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(max_yaw_speed > 0.0f);
    RT_ASSERT(max_pitch_speed > 0.0f);
    RT_ASSERT(acceleration > 0.0f);
    RT_ASSERT(smooth_factor >= 0.0f && smooth_factor <= 1.0f);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->motion_params.max_yaw_speed = max_yaw_speed;
    gimbal->motion_params.max_pitch_speed = max_pitch_speed;
    gimbal->motion_params.acceleration = acceleration;
    gimbal->motion_params.smooth_factor = smooth_factor;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Motion params set: yaw_speed=%.1f°/s, pitch_speed=%.1f°/s, accel=%.1f°/s², smooth=%.2f",
          max_yaw_speed, max_pitch_speed, acceleration, smooth_factor);
    return RT_EOK;
}

/**
 * @brief 设置云台角度限制
 */
rt_err_t laser_gimbal_set_angle_limits(laser_gimbal_t *gimbal,
                                       float yaw_min, float yaw_max,
                                       float pitch_min, float pitch_max)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(yaw_max > yaw_min);
    RT_ASSERT(pitch_max > pitch_min);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->angle_limits_min.yaw = yaw_min;
    gimbal->angle_limits_max.yaw = yaw_max;
    gimbal->angle_limits_min.pitch = pitch_min;
    gimbal->angle_limits_max.pitch = pitch_max;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Angle limits set: yaw[%.1f°,%.1f°], pitch[%.1f°,%.1f°]",
          yaw_min, yaw_max, pitch_min, pitch_max);
    return RT_EOK;
}

/**
 * @brief 坐标控制 - 输入距离和XY坐标控制激光指向
 */
rt_err_t laser_gimbal_point_to_coordinate(laser_gimbal_t *gimbal,
                                          float distance,
                                          float x, float y)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 参数验证 */
    if (distance < LASER_GIMBAL_MIN_DISTANCE || distance > LASER_GIMBAL_MAX_DISTANCE)
    {
        LOG_E("Distance %.2fm out of range [%.1f, %.1f]",
              distance, LASER_GIMBAL_MIN_DISTANCE, LASER_GIMBAL_MAX_DISTANCE);
        return -RT_EINVAL;
    }

    /* 构造三维坐标 */
    laser_coordinate_t target;
    target.x = x;
    target.y = y;
    target.z = distance; // 使用距离作为斜距

    /* 验证坐标 */
    rt_err_t result = laser_gimbal_validate_coordinate(target);
    if (result != RT_EOK)
    {
        return result;
    }

    return laser_gimbal_point_to_3d_coordinate(gimbal, target);
}

/**
 * @brief 三维坐标控制 - 输入三维坐标控制激光指向
 */
rt_err_t laser_gimbal_point_to_3d_coordinate(laser_gimbal_t *gimbal,
                                             laser_coordinate_t target)
{
    RT_ASSERT(gimbal != RT_NULL);

    rt_err_t result = RT_EOK;
    laser_gimbal_angle_t target_angle;

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 坐标转换为角度 */
    result = laser_gimbal_coordinate_to_angle(target, gimbal->gimbal_height, &target_angle);
    if (result != RT_EOK)
    {
        rt_mutex_release(gimbal->gimbal_mutex);
        LOG_E("Failed to convert coordinate to angle");
        return result;
    }

    /* 检查角度限制 */
    if (!laser_gimbal_check_angle_limits(gimbal, target_angle))
    {
        rt_mutex_release(gimbal->gimbal_mutex);
        LOG_E("Target angle out of limits: yaw=%.2f°, pitch=%.2f°",
              target_angle.yaw, target_angle.pitch);
        return -RT_EINVAL;
    }

    /* 更新目标 */
    gimbal->current_target = target;
    gimbal->target_angle = target_angle;
    gimbal->mode = LASER_GIMBAL_MODE_COORDINATE;

    rt_mutex_release(gimbal->gimbal_mutex);

    /* 执行运动 */
    result = laser_gimbal_move_to_angle(gimbal, target_angle);
    if (result == RT_EOK)
    {
        gimbal->command_count++;
        LOG_I("Pointing to coordinate: (%.2f, %.2f, %.2f) -> angle: (%.2f°, %.2f°)",
              target.x, target.y, target.z, target_angle.yaw, target_angle.pitch);
    }
    else
    {
        gimbal->error_count++;
    }

    return result;
}

/**
 * @brief 直接角度控制
 */
rt_err_t laser_gimbal_set_angle(laser_gimbal_t *gimbal,
                                float yaw, float pitch)
{
    RT_ASSERT(gimbal != RT_NULL);

    laser_gimbal_angle_t target_angle = {yaw, pitch};

    /* 检查角度限制 */
    if (!laser_gimbal_check_angle_limits(gimbal, target_angle))
    {
        LOG_E("Angle out of limits: yaw=%.2f°, pitch=%.2f°", yaw, pitch);
        return -RT_EINVAL;
    }

    // if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    // {
    //     return -RT_ERROR;
    // }

    gimbal->target_angle = target_angle;
    gimbal->mode = LASER_GIMBAL_MODE_DIRECT;

    //rt_mutex_release(gimbal->gimbal_mutex);

    rt_err_t result = laser_gimbal_move_to_angle(gimbal, target_angle);
    if (result == RT_EOK)
    {
        gimbal->command_count++;
       // LOG_I("Set angle: yaw=%.2f°, pitch=%.2f°", yaw, pitch);
    }
    else
    {
        gimbal->error_count++;
    }

    return result;
}

/**
 * @brief 获取当前云台角度
 */
rt_err_t laser_gimbal_get_current_angle(laser_gimbal_t *gimbal,
                                        laser_gimbal_angle_t *angle)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(angle != RT_NULL);

    // if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    // {
    //     return -RT_ERROR;
    // }

    /* 更新当前位置 */
    laser_gimbal_update_current_position(gimbal);

    *angle = gimbal->current_angle;

    //rt_mutex_release(gimbal->gimbal_mutex);

    return RT_EOK;
}

/* ======================== 便利的角度控制函数 ======================== */

/**
 * @brief 设置单独的yaw角度 (保持当前pitch不变)
 */
rt_err_t laser_gimbal_set_yaw(laser_gimbal_t *gimbal, float yaw)
{
    RT_ASSERT(gimbal != RT_NULL);

    laser_gimbal_angle_t current_angle;
    rt_err_t result = laser_gimbal_get_current_angle(gimbal, &current_angle);
    if (result != RT_EOK)
    {
        return result;
    }

    return laser_gimbal_set_angle(gimbal, yaw, current_angle.pitch);
}

/**
 * @brief 设置单独的pitch角度 (保持当前yaw不变)
 */
rt_err_t laser_gimbal_set_pitch(laser_gimbal_t *gimbal, float pitch)
{
    RT_ASSERT(gimbal != RT_NULL);

    laser_gimbal_angle_t current_angle;
    rt_err_t result = laser_gimbal_get_current_angle(gimbal, &current_angle);
    if (result != RT_EOK)
    {
        return result;
    }

    return laser_gimbal_set_angle(gimbal, 0, pitch);
}

/**
 * @brief 相对角度调整 (在当前角度基础上增减)
 */
rt_err_t laser_gimbal_adjust_angle(laser_gimbal_t *gimbal, 
                                  float yaw_delta, float pitch_delta)
{
    RT_ASSERT(gimbal != RT_NULL);

    laser_gimbal_angle_t current_angle;
    rt_err_t result = laser_gimbal_get_current_angle(gimbal, &current_angle);
    if (result != RT_EOK)
    {
        return result;
    }

    float new_yaw = current_angle.yaw + yaw_delta;
    float new_pitch = current_angle.pitch + pitch_delta;

    // LOG_I("Adjusting angle: yaw %+.2f° (%.2f°→%.2f°), pitch %+.2f° (%.2f°→%.2f°)",
    //       yaw_delta, current_angle.yaw, new_yaw,
    //       pitch_delta, current_angle.pitch, new_pitch);

    return laser_gimbal_set_angle(gimbal, new_yaw, new_pitch);
}

/**
 * @brief 平滑移动到指定角度 (可控制速度)
 */
rt_err_t laser_gimbal_smooth_move_to_angle(laser_gimbal_t *gimbal,
                                          float yaw, float pitch, 
                                          float speed_factor)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 参数验证 */
    if (speed_factor < 0.1f || speed_factor > 2.0f)
    {
        LOG_W("Speed factor %.2f out of range [0.1-2.0], using 1.0", speed_factor);
        speed_factor = 1.0f;
    }

    laser_gimbal_angle_t target_angle = {yaw, pitch};

    /* 检查角度限制 */
    if (!laser_gimbal_check_angle_limits(gimbal, target_angle))
    {
        LOG_E("Angle out of limits: yaw=%.2f°, pitch=%.2f°", yaw, pitch);
        return -RT_EINVAL;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 临时调整运动参数 */
    laser_gimbal_motion_params_t original_params = gimbal->motion_params;
    gimbal->motion_params.max_yaw_speed *= speed_factor;
    gimbal->motion_params.max_pitch_speed *= speed_factor;
    gimbal->motion_params.acceleration *= speed_factor;

    gimbal->target_angle = target_angle;
    gimbal->mode = LASER_GIMBAL_MODE_SMOOTH;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Smooth move to angle: yaw=%.2f°, pitch=%.2f°, speed=%.1fx", 
          yaw, pitch, speed_factor);

    rt_err_t result = laser_gimbal_move_to_angle(gimbal, target_angle);

    /* 恢复原始运动参数 */
    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) == RT_EOK)
    {
        gimbal->motion_params = original_params;
        rt_mutex_release(gimbal->gimbal_mutex);
    }

    if (result == RT_EOK)
    {
        gimbal->command_count++;
    }
    else
    {
        gimbal->error_count++;
    }

    return result;
}

/**
 * @brief 获取当前yaw角度
 */
rt_err_t laser_gimbal_get_yaw(laser_gimbal_t *gimbal, float *yaw)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(yaw != RT_NULL);

    laser_gimbal_angle_t current_angle;
    rt_err_t result = laser_gimbal_get_current_angle(gimbal, &current_angle);
    if (result == RT_EOK)
    {
        *yaw = current_angle.yaw;
    }

    return result;
}

/**
 * @brief 获取当前pitch角度
 */
rt_err_t laser_gimbal_get_pitch(laser_gimbal_t *gimbal, float *pitch)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(pitch != RT_NULL);

    laser_gimbal_angle_t current_angle;
    rt_err_t result = laser_gimbal_get_current_angle(gimbal, &current_angle);
    if (result == RT_EOK)
    {
        *pitch = current_angle.pitch;
    }

    return result;
}

/* ======================== 速度控制函数实现 ======================== */

/**
 * @brief 设置yaw轴旋转速度 (连续旋转)
 */
rt_err_t laser_gimbal_set_yaw_speed(laser_gimbal_t *gimbal, float speed)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 限制速度范围 */
    if (fabsf(speed) > gimbal->motion_params.max_yaw_speed)
    {
        LOG_W("Yaw speed %.1f°/s exceeds limit %.1f°/s", 
              speed, gimbal->motion_params.max_yaw_speed);
        speed = (speed > 0) ? gimbal->motion_params.max_yaw_speed : -gimbal->motion_params.max_yaw_speed;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->velocity_control_enabled = true;
    gimbal->current_yaw_speed = speed;
    gimbal->velocity_update_time = rt_tick_get_millisecond();

    rt_mutex_release(gimbal->gimbal_mutex);

    /* 转换为电机速度单位 */
    rt_int32_t motor_speed = (rt_int32_t)(speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    
    /* 应用机械参数修正 */
    if (gimbal->yaw_reverse)
        motor_speed = -motor_speed;

    /* 设置电机速度控制 */
    rt_err_t result = ms4010_speed_control(gimbal->yaw_motor, motor_speed);
    
    if (result == RT_EOK)
    {
        LOG_D("Yaw speed set: %.1f°/s (motor: %d)", speed, motor_speed);
    }
    else
    {
        gimbal->error_count++;
        LOG_E("Failed to set yaw speed: %d", result);
    }

    return result;
}

/**
 * @brief 设置pitch轴旋转速度 (连续旋转)
 */
rt_err_t laser_gimbal_set_pitch_speed(laser_gimbal_t *gimbal, float speed)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 限制速度范围 */
    if (fabsf(speed) > gimbal->motion_params.max_pitch_speed)
    {
        LOG_W("Pitch speed %.1f°/s exceeds limit %.1f°/s", 
              speed, gimbal->motion_params.max_pitch_speed);
        speed = (speed > 0) ? gimbal->motion_params.max_pitch_speed : -gimbal->motion_params.max_pitch_speed;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->velocity_control_enabled = true;
    gimbal->current_pitch_speed = speed;
    gimbal->velocity_update_time = rt_tick_get_millisecond();

    rt_mutex_release(gimbal->gimbal_mutex);

    /* 转换为电机速度单位 */
    rt_int32_t motor_speed = (rt_int32_t)(speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    
    /* 应用机械参数修正 */
    if (gimbal->pitch_reverse)
        motor_speed = -motor_speed;

    /* 设置电机速度控制 */
    rt_err_t result = ms4010_speed_control(gimbal->pitch_motor, motor_speed);
    
    if (result == RT_EOK)
    {
        LOG_D("Pitch speed set: %.1f°/s (motor: %d)", speed, motor_speed);
    }
    else
    {
        gimbal->error_count++;
        LOG_E("Failed to set pitch speed: %d", result);
    }

    return result;
}

/**
 * @brief 同时设置yaw和pitch轴旋转速度
 */
rt_err_t laser_gimbal_set_angular_velocity(laser_gimbal_t *gimbal, 
                                          float yaw_speed, float pitch_speed)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 限制速度范围 */
    if (fabsf(yaw_speed) > gimbal->motion_params.max_yaw_speed)
    {
        LOG_W("Yaw speed %.1f°/s exceeds limit %.1f°/s", 
              yaw_speed, gimbal->motion_params.max_yaw_speed);
        yaw_speed = (yaw_speed > 0) ? gimbal->motion_params.max_yaw_speed : -gimbal->motion_params.max_yaw_speed;
    }

    if (fabsf(pitch_speed) > gimbal->motion_params.max_pitch_speed)
    {
        LOG_W("Pitch speed %.1f°/s exceeds limit %.1f°/s", 
              pitch_speed, gimbal->motion_params.max_pitch_speed);
        pitch_speed = (pitch_speed > 0) ? gimbal->motion_params.max_pitch_speed : -gimbal->motion_params.max_pitch_speed;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->velocity_control_enabled = true;
    gimbal->current_yaw_speed = yaw_speed;
    gimbal->current_pitch_speed = pitch_speed;
    gimbal->velocity_update_time = rt_tick_get_millisecond();

    rt_mutex_release(gimbal->gimbal_mutex);

    /* 转换为电机速度单位 */
    rt_int32_t yaw_motor_speed = (rt_int32_t)(yaw_speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    rt_int32_t pitch_motor_speed = (rt_int32_t)(pitch_speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    
    /* 应用机械参数修正 */
    if (gimbal->yaw_reverse)
        yaw_motor_speed = -yaw_motor_speed;
    if (gimbal->pitch_reverse)
        pitch_motor_speed = -pitch_motor_speed;

    /* 同时设置两个电机的速度控制 */
    rt_err_t result1 = ms4010_speed_control(gimbal->yaw_motor, yaw_motor_speed);
    rt_err_t result2 = ms4010_speed_control(gimbal->pitch_motor, pitch_motor_speed);
    
    rt_err_t result = (result1 == RT_EOK && result2 == RT_EOK) ? RT_EOK : -RT_ERROR;
    
    if (result == RT_EOK)
    {
        gimbal->command_count++;
       // LOG_I("Angular velocity set: yaw=%.1f°/s, pitch=%.1f°/s", yaw_speed, pitch_speed);
    }
    else
    {
        gimbal->error_count++;
        LOG_E("Failed to set angular velocity: yaw=%d, pitch=%d", result1, result2);
    }

    return result;
}

/**
 * @brief 获取当前yaw轴旋转速度
 */
rt_err_t laser_gimbal_get_yaw_speed(laser_gimbal_t *gimbal, float *speed)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(speed != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    *speed = gimbal->current_yaw_speed;

    rt_mutex_release(gimbal->gimbal_mutex);

    return RT_EOK;
}

/**
 * @brief 获取当前pitch轴旋转速度
 */
rt_err_t laser_gimbal_get_pitch_speed(laser_gimbal_t *gimbal, float *speed)
{
    RT_ASSERT(gimbal != RT_NULL);
    RT_ASSERT(speed != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    *speed = gimbal->current_pitch_speed;

    rt_mutex_release(gimbal->gimbal_mutex);

    return RT_EOK;
}

/**
 * @brief 停止所有轴的速度控制 (回到位置控制模式)
 */
rt_err_t laser_gimbal_stop_velocity_control(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->velocity_control_enabled = false;
    gimbal->current_yaw_speed = 0.0f;
    gimbal->current_pitch_speed = 0.0f;
    gimbal->velocity_update_time = rt_tick_get_millisecond();

    rt_mutex_release(gimbal->gimbal_mutex);

    /* 停止两个电机的速度控制 */
    rt_err_t result1 = ms4010_motor_stop(gimbal->yaw_motor);
    rt_err_t result2 = ms4010_motor_stop(gimbal->pitch_motor);
    
    rt_err_t result = (result1 == RT_EOK && result2 == RT_EOK) ? RT_EOK : -RT_ERROR;
    
    if (result == RT_EOK)
    {
        LOG_I("Velocity control stopped - returned to position control mode");
    }
    else
    {
        gimbal->error_count++;
        LOG_E("Failed to stop velocity control: yaw=%d, pitch=%d", result1, result2);
    }

    return result;
}

/* ======================== 基本控制函数 ======================== */

/**
 * @brief 回零操作
 */
rt_err_t laser_gimbal_home(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    LOG_I("Gimbal homing...");

    laser_gimbal_angle_t home_angle = {0.0f, 0.0f};
    return laser_gimbal_set_angle(gimbal, home_angle.yaw, home_angle.pitch);
}

/**
 * @brief 停止云台运动
 */
rt_err_t laser_gimbal_stop(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    rt_err_t result1 = ms4010_motor_stop(gimbal->yaw_motor);
    rt_err_t result2 = ms4010_motor_stop(gimbal->pitch_motor);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) == RT_EOK)
    {
        gimbal->state = LASER_GIMBAL_STATE_IDLE;
        
        /* 如果在速度控制模式，退出速度控制 */
        if (gimbal->velocity_control_enabled)
        {
            gimbal->velocity_control_enabled = false;
            gimbal->current_yaw_speed = 0.0f;
            gimbal->current_pitch_speed = 0.0f;
            gimbal->velocity_update_time = rt_tick_get_millisecond();
            LOG_I("Velocity control mode disabled");
        }
        
        rt_mutex_release(gimbal->gimbal_mutex);
    }

    LOG_I("Gimbal stopped");

    return (result1 == RT_EOK && result2 == RT_EOK) ? RT_EOK : -RT_ERROR;
}

/**
 * @brief 启用/禁用电机
 */
rt_err_t laser_gimbal_enable(laser_gimbal_t *gimbal, bool enable)
{
    RT_ASSERT(gimbal != RT_NULL);

    rt_err_t result1, result2;

    if (enable)
    {
        result1 = ms4010_motor_on(gimbal->yaw_motor);
        result2 = ms4010_motor_on(gimbal->pitch_motor);
        LOG_I("Gimbal motors enabled");
    }
    else
    {
        result1 = ms4010_motor_off(gimbal->yaw_motor);
        result2 = ms4010_motor_off(gimbal->pitch_motor);
        LOG_I("Gimbal motors disabled");
    }

    return (result1 == RT_EOK && result2 == RT_EOK) ? RT_EOK : -RT_ERROR;
}

/**
 * @brief 获取云台状态信息
 */
laser_gimbal_state_t laser_gimbal_get_state(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    laser_gimbal_state_t state;

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) == RT_EOK)
    {
        state = gimbal->state;
        rt_mutex_release(gimbal->gimbal_mutex);
    }
    else
    {
        state = LASER_GIMBAL_STATE_ERROR;
    }

    return state;
}

/**
 * @brief 获取云台统计信息
 */
rt_err_t laser_gimbal_get_statistics(laser_gimbal_t *gimbal,
                                     uint32_t *command_count,
                                     uint32_t *error_count)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (command_count != RT_NULL)
        *command_count = gimbal->command_count;

    if (error_count != RT_NULL)
        *error_count = gimbal->error_count;

    return RT_EOK;
}

/* 辅助函数实现 */

/**
 * @brief 坐标转角度计算
 */
rt_err_t laser_gimbal_coordinate_to_angle(laser_coordinate_t target,
                                          float gimbal_height,
                                          laser_gimbal_angle_t *angle)
{
    RT_ASSERT(angle != RT_NULL);

    float x = target.x;
    float y = target.y;
    float z = target.z;

    //    /* 计算水平距离 */
    //    float horizontal_distance = sqrtf(x*x + y*y);
    //
    //    /* 计算偏航角 (方位角) */
    //    if (horizontal_distance < 1e-6f) // 接近原点时
    //    {
    //        angle->yaw = 0.0f;  // home点，偏航角为0
    //    }
    //    else
    //    {
    angle->yaw = LASER_GIMBAL_RAD_TO_DEG(atan2f(x, z));
    //    }

    angle->pitch = LASER_GIMBAL_RAD_TO_DEG(atan2f(y, z));

    /* 角度归一化 */
    angle->yaw = laser_gimbal_normalize_angle(angle->yaw);
    angle->pitch = laser_gimbal_normalize_angle(angle->pitch);

    LOG_D("Coordinate (%.2f,%.2f,%.2f) -> Angle (%.2f°,%.2f°)",
          x, y, z, angle->yaw, angle->pitch);

    return RT_EOK;
}

/**
 * @brief 角度转坐标计算 (用于验证)
 */
rt_err_t laser_gimbal_angle_to_coordinate(laser_gimbal_angle_t angle,
                                          float distance,
                                          float gimbal_height,
                                          laser_coordinate_t *coordinate)
{
    RT_ASSERT(coordinate != RT_NULL);

    float yaw_rad = LASER_GIMBAL_DEG_TO_RAD(angle.yaw);
    float pitch_rad = LASER_GIMBAL_DEG_TO_RAD(angle.pitch);

    /* 计算坐标 */
    float horizontal_distance = distance * cosf(pitch_rad);
    coordinate->x = horizontal_distance * cosf(yaw_rad);
    coordinate->y = horizontal_distance * sinf(yaw_rad);
    coordinate->z = gimbal_height - distance * sinf(pitch_rad);

    return RT_EOK;
}

/**
 * @brief 角度限制检查
 */
bool laser_gimbal_check_angle_limits(laser_gimbal_t *gimbal,
                                     laser_gimbal_angle_t angle)
{
    RT_ASSERT(gimbal != RT_NULL);

    return (angle.yaw >= gimbal->angle_limits_min.yaw &&
            angle.yaw <= gimbal->angle_limits_max.yaw &&
            angle.pitch >= gimbal->angle_limits_min.pitch &&
            angle.pitch <= gimbal->angle_limits_max.pitch);
}

/* 内部函数实现 */

/**
 * @brief 移动到指定角度
 */
static rt_err_t laser_gimbal_move_to_angle(laser_gimbal_t *gimbal, laser_gimbal_angle_t target_angle)
{
    RT_ASSERT(gimbal != RT_NULL);

    rt_err_t result1, result2;

    // if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    // {
    //     return -RT_ERROR;
    // }

    gimbal->state = LASER_GIMBAL_STATE_MOVING;

    /* 应用机械参数修正 */
    float yaw_angle = target_angle.yaw + gimbal->yaw_offset;
    float pitch_angle = target_angle.pitch + gimbal->pitch_offset;

    if (gimbal->yaw_reverse)
        yaw_angle = -yaw_angle;

    if (gimbal->pitch_reverse)
        pitch_angle = -pitch_angle;

    //rt_mutex_release(gimbal->gimbal_mutex);

    /* 转换为电机角度单位 (MS4010使用0.01度单位) */
    rt_int32_t yaw_motor_angle = (rt_int32_t)(yaw_angle * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    rt_int32_t pitch_motor_angle = (rt_int32_t)(pitch_angle * LASER_GIMBAL_MOTOR_ANGLE_SCALE);

    /* 计算速度限制 */
    rt_uint32_t yaw_speed = (rt_uint32_t)(gimbal->motion_params.max_yaw_speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    rt_uint32_t pitch_speed = (rt_uint32_t)(gimbal->motion_params.max_pitch_speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);

    /* 同时控制两个电机移动到目标位置 */
    result1 = ms4010_position_control2(gimbal->yaw_motor, yaw_motor_angle, yaw_speed);
    result2 = ms4010_position_control2(gimbal->pitch_motor, pitch_motor_angle, pitch_speed);

    /* 更新当前角度 */
    if (result1 == RT_EOK && result2 == RT_EOK)
    {
        // if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) == RT_EOK)
        // {
            gimbal->current_angle = target_angle;
            gimbal->state = LASER_GIMBAL_STATE_IDLE;
            gimbal->last_update_time = rt_tick_get();
        //     rt_mutex_release(gimbal->gimbal_mutex);
        // }

        /* 调用到达目标回调 */
        if (gimbal->on_target_reached != RT_NULL)
        {
            gimbal->on_target_reached(gimbal->current_target);
        }
    }
    else
    {
        // if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) == RT_EOK)
        // {
           // gimbal->state = LASER_GIMBAL_STATE_ERROR;
        //     rt_mutex_release(gimbal->gimbal_mutex);
        // }

        /* 调用错误回调 */
        if (gimbal->on_error != RT_NULL)
        {
            gimbal->on_error((result1 != RT_EOK) ? 1 : 2);
        }

        LOG_E("Failed to move motors: yaw=%d, pitch=%d", result1, result2);
    }

    return (result1 == RT_EOK && result2 == RT_EOK) ? RT_EOK : -RT_ERROR;
}

/**
 * @brief 更新当前位置 (从电机读取)
 */
static rt_err_t laser_gimbal_update_current_position(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 这里可以从电机读取当前位置，暂时使用目标角度 */
    /* 实际项目中可以读取电机的位置反馈 */

    // TODO: 从MS4010电机读取当前位置
    // ms4010_status_t yaw_status, pitch_status;
    // ms4010_get_status(gimbal->yaw_motor, &yaw_status);
    // ms4010_get_status(gimbal->pitch_motor, &pitch_status);
    //
    gimbal->current_angle.yaw = gimbal->target_angle.yaw;
    gimbal->current_angle.pitch = gimbal->target_angle.pitch;

    return RT_EOK;
}

/**
 * @brief 角度归一化 (-180° 到 +180°)
 */
static float laser_gimbal_normalize_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

/**
 * @brief 验证坐标参数
 */
static rt_err_t laser_gimbal_validate_coordinate(laser_coordinate_t coordinate)
{
    /* 检查坐标范围 */
    float distance = sqrtf(coordinate.x * coordinate.x + coordinate.y * coordinate.y + coordinate.z * coordinate.z);

    if (distance < LASER_GIMBAL_MIN_DISTANCE)
    {
        LOG_E("Target too close: %.2fm < %.2fm", distance, LASER_GIMBAL_MIN_DISTANCE);
        return -RT_EINVAL;
    }

    if (distance > LASER_GIMBAL_MAX_DISTANCE)
    {
        LOG_E("Target too far: %.2fm > %.2fm", distance, LASER_GIMBAL_MAX_DISTANCE);
        return -RT_EINVAL;
    }

    return RT_EOK;
}

/* 角度校准功能实现 */

/**
 * @brief 进入校准模式 - 关闭电机，启动角度监控
 */
rt_err_t laser_gimbal_calibration_mode(laser_gimbal_t *gimbal, bool enable)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->calibration_mode = enable;

    if (enable)
    {
        gimbal->state = LASER_GIMBAL_STATE_IDLE;

        /* 关闭电机，允许手动调整 */
        if (gimbal->yaw_motor)
        {
            ms4010_motor_off(gimbal->yaw_motor);
        }
        if (gimbal->pitch_motor)
        {
            ms4010_motor_off(gimbal->pitch_motor);
        }

        /* 启动角度监控 */
        laser_gimbal_start_calibration_monitor(gimbal);

        LOG_I("Calibration mode ENABLED:");
        LOG_I("  - Motors are OFF (manual adjustment allowed)");
        LOG_I("  - Real-time angle monitoring started");
        LOG_I("  - Auto-save when angles stable for %.1fs",
              gimbal->angle_stable_time_ms / 1000.0f);
        LOG_I("Use your hands to adjust gimbal to desired position");
    }
    else
    {
        /* 停止角度监控 */
        laser_gimbal_stop_calibration_monitor(gimbal);

        /* 重新启用电机 */
        if (gimbal->yaw_motor)
        {
            ms4010_motor_on(gimbal->yaw_motor);
        }
        if (gimbal->pitch_motor)
        {
            ms4010_motor_on(gimbal->pitch_motor);
        }

        LOG_I("Calibration mode DISABLED - Normal operation resumed");
        LOG_I("  - Motors are re-enabled");
        LOG_I("  - Angle monitoring stopped");
    }

    rt_mutex_release(gimbal->gimbal_mutex);

    return RT_EOK;
}

/**
 * @brief 校准模式下直接设置电机角度 (不应用偏移)
 */
rt_err_t laser_gimbal_calibration_set_motor_angle(laser_gimbal_t *gimbal,
                                                  float yaw_motor_angle,
                                                  float pitch_motor_angle)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (!gimbal->calibration_mode)
    {
        LOG_E("Not in calibration mode! Use laser_gimbal_calibration_mode(true) first");
        return -RT_ERROR;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->state = LASER_GIMBAL_STATE_MOVING;

    rt_mutex_release(gimbal->gimbal_mutex);

    /* 转换为电机角度单位 (MS4010使用0.01度单位) */
    rt_int32_t yaw_motor_value = (rt_int32_t)(yaw_motor_angle * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    rt_int32_t pitch_motor_value = (rt_int32_t)(pitch_motor_angle * LASER_GIMBAL_MOTOR_ANGLE_SCALE);

    /* 计算速度限制 */
    rt_uint32_t yaw_speed = (rt_uint32_t)(gimbal->motion_params.max_yaw_speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);
    rt_uint32_t pitch_speed = (rt_uint32_t)(gimbal->motion_params.max_pitch_speed * LASER_GIMBAL_MOTOR_ANGLE_SCALE);

    /* 直接控制电机，不应用偏移和反向 */
    rt_err_t result1 = ms4010_position_control2(gimbal->yaw_motor, yaw_motor_value, yaw_speed);
    rt_err_t result2 = ms4010_position_control2(gimbal->pitch_motor, pitch_motor_value, pitch_speed);

    if (result1 == RT_EOK && result2 == RT_EOK)
    {
        if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            gimbal->yaw_motor_current_angle = yaw_motor_angle;
            gimbal->pitch_motor_current_angle = pitch_motor_angle;
            gimbal->state = LASER_GIMBAL_STATE_IDLE;
            rt_mutex_release(gimbal->gimbal_mutex);
        }

        LOG_I("Motor angles set: Yaw=%.2f°, Pitch=%.2f°", yaw_motor_angle, pitch_motor_angle);
    }
    else
    {
        if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            gimbal->state = LASER_GIMBAL_STATE_ERROR;
            rt_mutex_release(gimbal->gimbal_mutex);
        }
        LOG_E("Failed to set motor angles: yaw=%d, pitch=%d", result1, result2);
    }

    return (result1 == RT_EOK && result2 == RT_EOK) ? RT_EOK : -RT_ERROR;
}

/**
 * @brief 校准模式下微调电机角度
 */
rt_err_t laser_gimbal_calibration_adjust_angle(laser_gimbal_t *gimbal,
                                               float yaw_delta,
                                               float pitch_delta)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (!gimbal->calibration_mode)
    {
        LOG_E("Not in calibration mode! Use laser_gimbal_calibration_mode(true) first");
        return -RT_ERROR;
    }

    /* 从电机读取当前实际角度 */
    float current_yaw, current_pitch;
    rt_err_t result = laser_gimbal_get_motor_raw_angle(gimbal, &current_yaw, &current_pitch);
    if (result != RT_EOK)
    {
        LOG_E("Failed to read current motor angles");
        return result;
    }

    float new_yaw = current_yaw + yaw_delta;
    float new_pitch = current_pitch + pitch_delta;

    LOG_I("Adjusting angles: Yaw %.2f° %+.2f° -> %.2f°, Pitch %.2f° %+.2f° -> %.2f°",
          current_yaw, yaw_delta, new_yaw, current_pitch, pitch_delta, new_pitch);

    return laser_gimbal_calibration_set_motor_angle(gimbal, new_yaw, new_pitch);
}

/**
 * @brief 获取当前电机原始角度 (用于校准)
 */
rt_err_t laser_gimbal_get_motor_raw_angle(laser_gimbal_t *gimbal,
                                          float *yaw_motor_angle,
                                          float *pitch_motor_angle)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_err_t result = RT_EOK;

    /* 从MS4010电机读取当前实际位置 */
    if (yaw_motor_angle != RT_NULL)
    {
        ms4010_status_t motor_status;
        rt_err_t yaw_result = ms4010_get_status(gimbal->yaw_motor, &motor_status);
        if (yaw_result == RT_EOK)
        {
            // 编码器值转换为角度（假设编码器范围0-65535对应360度）
            *yaw_motor_angle = ((float)motor_status.encoder / 65535.0f) * 360.0f;
            gimbal->yaw_motor_current_angle = *yaw_motor_angle; // 更新内部记录
        }
        else
        {
            LOG_W("Failed to read yaw motor position, using cached value");
            *yaw_motor_angle = gimbal->yaw_motor_current_angle;
            result = yaw_result;
        }
    }

    if (pitch_motor_angle != RT_NULL)
    {
        ms4010_status_t motor_status;
        rt_err_t pitch_result = ms4010_get_status(gimbal->pitch_motor, &motor_status);
        if (pitch_result == RT_EOK)
        {
            // 编码器值转换为角度（假设编码器范围0-65535对应360度）
            *pitch_motor_angle = ((float)motor_status.encoder / 65535.0f) * 360.0f;
            gimbal->pitch_motor_current_angle = *pitch_motor_angle; // 更新内部记录
        }
        else
        {
            LOG_W("Failed to read pitch motor position, using cached value");
            *pitch_motor_angle = gimbal->pitch_motor_current_angle;
            result = pitch_result;
        }
    }

    rt_mutex_release(gimbal->gimbal_mutex);

    return result;
}

/**
 * @brief 计算并设置角度偏移 (当云台指向期望角度位置时调用)
 */
rt_err_t laser_gimbal_calibrate_offset(laser_gimbal_t *gimbal,
                                       float target_yaw,
                                       float target_pitch)
{
    RT_ASSERT(gimbal != RT_NULL);

//    if (!gimbal->calibration_mode)
//    {
//        LOG_E("Not in calibration mode! Please adjust gimbal to target position first");
//        return -RT_ERROR;
//    }

    /* 首先从电机读取当前真实角度 */
    float current_yaw_motor, current_pitch_motor;
    rt_err_t result = laser_gimbal_get_motor_raw_angle(gimbal, &current_yaw_motor, &current_pitch_motor);
    if (result != RT_EOK)
    {
        LOG_E("Failed to read motor angles for calibration");
        return result;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 计算偏移量
     * 正常控制时: 电机角度 = (逻辑角度 + 偏移量) × 反向系数
     * 校准时已知: 当前电机角度, 期望逻辑角度
     * 求解偏移量: 偏移量 = (当前电机角度 / 反向系数) - 期望逻辑角度
     */

    float yaw_motor_corrected = current_yaw_motor;
    float pitch_motor_corrected = current_pitch_motor;

    /* 处理反向 */
    if (gimbal->yaw_reverse)
        yaw_motor_corrected = -yaw_motor_corrected;
    if (gimbal->pitch_reverse)
        pitch_motor_corrected = -pitch_motor_corrected;

    /* 计算偏移量 */
    float yaw_offset_new = yaw_motor_corrected - target_yaw;
    float pitch_offset_new = pitch_motor_corrected - target_pitch;

    gimbal->yaw_offset = yaw_offset_new;
    gimbal->pitch_offset = pitch_offset_new;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("✅ Calibration completed!");
    LOG_I("  Target logic angles: Yaw=%.2f°, Pitch=%.2f°", target_yaw, target_pitch);
    LOG_I("  Current motor angles: Yaw=%.2f°, Pitch=%.2f°", current_yaw_motor, current_pitch_motor);
    LOG_I("  Motor corrected (after reverse): Yaw=%.2f°, Pitch=%.2f°", yaw_motor_corrected, pitch_motor_corrected);
    LOG_I("  Calculated offsets: Yaw=%.2f°, Pitch=%.2f°", yaw_offset_new, pitch_offset_new);
    LOG_I("💾 Use 'laser_cal_save' to save these settings");

    return RT_EOK;
}

/**
 * @brief 保存校准参数到文件
 */
/**
 * @brief 保存校准参数到变量存储
 */
rt_err_t laser_gimbal_save_calibration(laser_gimbal_t *gimbal, const char *filename)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 将校准参数保存到全局变量 */
    g_calibration_storage.is_valid = true;
    g_calibration_storage.yaw_offset = gimbal->yaw_offset;
    g_calibration_storage.pitch_offset = gimbal->pitch_offset;
    g_calibration_storage.yaw_reverse = gimbal->yaw_reverse;
    g_calibration_storage.pitch_reverse = gimbal->pitch_reverse;
    g_calibration_storage.gimbal_height = gimbal->gimbal_height;
    g_calibration_storage.save_time = rt_tick_get();

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("✅ 校准参数已保存到变量存储");
    LOG_I("  偏航偏移: %.3f°, 俯仰偏移: %.3f°", gimbal->yaw_offset, gimbal->pitch_offset);
    LOG_I("  偏航反向: %s, 俯仰反向: %s",
          gimbal->yaw_reverse ? "是" : "否", gimbal->pitch_reverse ? "是" : "否");
    LOG_I("  云台高度: %.2fm", gimbal->gimbal_height);

    return RT_EOK;
}

/**
 * @brief 从变量存储加载校准参数
 */
rt_err_t laser_gimbal_load_calibration(laser_gimbal_t *gimbal, const char *filename)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 检查是否有有效的校准数据 */
    if (!g_calibration_storage.is_valid)
    {
        LOG_W("没有找到有效的校准参数");
        LOG_I("请先使用 'laser_cal_save' 保存校准参数");
    gimbal->yaw_offset = 97.07f;
    gimbal->pitch_offset = 17.76f;

        return -RT_ERROR;
    }

    /* 应用校准参数 */
    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->yaw_offset = g_calibration_storage.yaw_offset;
    gimbal->pitch_offset = g_calibration_storage.pitch_offset;
    gimbal->yaw_reverse = g_calibration_storage.yaw_reverse;
    gimbal->pitch_reverse = g_calibration_storage.pitch_reverse;
    gimbal->gimbal_height = g_calibration_storage.gimbal_height;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("✅ 校准参数已从变量存储加载");
    LOG_I("  偏航偏移: %.3f°, 俯仰偏移: %.3f°", gimbal->yaw_offset, gimbal->pitch_offset);
    LOG_I("  偏航反向: %s, 俯仰反向: %s",
          gimbal->yaw_reverse ? "是" : "否", gimbal->pitch_reverse ? "是" : "否");
    LOG_I("  云台高度: %.2fm", gimbal->gimbal_height);
    LOG_I("  保存时间: %u ticks", g_calibration_storage.save_time);

    return RT_EOK;
}

/**
 * @brief 查看校准参数存储状态
 */
rt_err_t laser_gimbal_show_calibration_storage(void)
{
    if (!g_calibration_storage.is_valid)
    {
        LOG_I("❌ 校准参数存储为空");
        return -RT_ERROR;
    }

    LOG_I("📊 校准参数存储状态:");
    LOG_I("  状态: ✅ 有效");
    LOG_I("  偏航偏移: %.3f°", g_calibration_storage.yaw_offset);
    LOG_I("  俯仰偏移: %.3f°", g_calibration_storage.pitch_offset);
    LOG_I("  偏航反向: %s", g_calibration_storage.yaw_reverse ? "是" : "否");
    LOG_I("  俯仰反向: %s", g_calibration_storage.pitch_reverse ? "是" : "否");
    LOG_I("  云台高度: %.2fm", g_calibration_storage.gimbal_height);
    LOG_I("  保存时间: %u ticks ago", rt_tick_get() - g_calibration_storage.save_time);

    return RT_EOK;
}

/**
 * @brief 重置校准参数存储
 */
rt_err_t laser_gimbal_reset_calibration_storage(void)
{
    g_calibration_storage.is_valid = false;
    g_calibration_storage.yaw_offset = 0.f;
    g_calibration_storage.pitch_offset = 0.f;
    g_calibration_storage.yaw_reverse = false;
    g_calibration_storage.pitch_reverse = false;
    g_calibration_storage.gimbal_height = 1.5f;
    g_calibration_storage.save_time = 0;

    LOG_I("🗑️ 校准参数存储已重置");
    return RT_EOK;
}

/* ======================== 校准监控功能实现 ======================== */

/**
 * @brief 校准角度监控线程入口函数
 */
static void calibration_monitor_thread_entry(void *parameter)
{
    laser_gimbal_t *gimbal = (laser_gimbal_t *)parameter;
    uint32_t last_print_time = 0;
    const uint32_t print_interval = 1000; // 每秒打印一次角度信息

    LOG_I("Calibration monitor thread started");

    while (gimbal->calibration_monitor_running)
    {
        float current_yaw, current_pitch;

        /* 读取当前电机角度 */
        if (laser_gimbal_get_motor_raw_angle(gimbal, &current_yaw, &current_pitch) == RT_EOK)
        {
            uint32_t current_time = rt_tick_get_millisecond();

            /* 检查角度是否稳定 */
            float yaw_diff = fabsf(current_yaw - gimbal->last_stable_yaw);
            float pitch_diff = fabsf(current_pitch - gimbal->last_stable_pitch);
            float max_diff = (yaw_diff > pitch_diff) ? yaw_diff : pitch_diff;

            if (max_diff > gimbal->angle_stable_threshold)
            {
                /* 角度发生变化，重置稳定计时器 */
                gimbal->last_angle_change_time = current_time;
                gimbal->last_stable_yaw = current_yaw;
                gimbal->last_stable_pitch = current_pitch;
            }
            else
            {
                /* 角度稳定，检查是否达到稳定时间要求 */
                uint32_t stable_duration = current_time - gimbal->last_angle_change_time;

                if (stable_duration >= gimbal->angle_stable_time_ms && gimbal->auto_save_enabled)
                {
                    /* 角度稳定时间足够，触发自动保存 */
                    LOG_I("Angles stable for %.1fs - Auto-saving calibration...",
                          stable_duration / 1000.0f);

                    /* 计算并设置偏移量 (假设当前位置为0°, 0°) */

                    laser_gimbal_calibrate_offset(gimbal, 0.f, 0.f);

                    /* 保存校准参数 */
                    laser_gimbal_save_calibration(gimbal, RT_NULL);

                    LOG_I("✅ Auto-calibration completed!");
                    LOG_I("   Final angles: Yaw=%.2f°, Pitch=%.2f°", current_yaw, current_pitch);
                    LOG_I("   Calculated offsets: Yaw=%.2f°, Pitch=%.2f°",
                          gimbal->yaw_offset, gimbal->pitch_offset);

                    /* 重置稳定计时器，避免重复保存 */
                    gimbal->last_angle_change_time = current_time;
                }
            }

            /* 定期打印角度信息 */
            if (current_time - last_print_time >= print_interval)
            {
                uint32_t stable_duration = current_time - gimbal->last_angle_change_time;
                LOG_D("📐 Current angles: Yaw=%.2f°, Pitch=%.2f° ", current_yaw, current_pitch);

                if (stable_duration < gimbal->angle_stable_time_ms)
                {
                    LOG_D("(Stable: %.1fs/%.1fs)\n",
                          stable_duration / 1000.0f,
                          gimbal->angle_stable_time_ms / 1000.0f);
                }
                else
                {
                    LOG_D("(✅ STABLE - %s)\n",
                          gimbal->auto_save_enabled ? "Will auto-save" : "Manual save required");
                }

                last_print_time = current_time;
            }
        }

        /* 休眠100ms */
        rt_thread_mdelay(100);
    }

    LOG_I("Calibration monitor thread stopped");
}

/**
 * @brief 设置校准监控参数
 */
rt_err_t laser_gimbal_set_calibration_monitor_params(laser_gimbal_t *gimbal,
                                                     float stable_threshold,
                                                     uint32_t stable_time_ms,
                                                     bool auto_save)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->angle_stable_threshold = stable_threshold;
    gimbal->angle_stable_time_ms = stable_time_ms;
    gimbal->auto_save_enabled = auto_save;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Calibration monitor parameters updated:");
    LOG_I("  Stable threshold: %.2f°", stable_threshold);
    LOG_I("  Stable time: %.1fs", stable_time_ms / 1000.0f);
    LOG_I("  Auto-save: %s", auto_save ? "Enabled" : "Disabled");

    return RT_EOK;
}

/**
 * @brief 启动校准角度监控
 */
rt_err_t laser_gimbal_start_calibration_monitor(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (gimbal->calibration_monitor_running)
    {
        LOG_W("Calibration monitor already running");
        return RT_EOK;
    }

    /* 初始化监控参数 */
    if (gimbal->angle_stable_threshold == 0.0f)
    {
        gimbal->angle_stable_threshold = 0.5f; // 默认0.5度稳定阈值
    }
    if (gimbal->angle_stable_time_ms == 0)
    {
        gimbal->angle_stable_time_ms = 3000; // 默认3秒稳定时间
    }
    if (!gimbal->auto_save_enabled)
    {
        gimbal->auto_save_enabled = true; // 默认启用自动保存
    }

    /* 初始化角度状态 */
    laser_gimbal_get_motor_raw_angle(gimbal, &gimbal->last_stable_yaw, &gimbal->last_stable_pitch);
    gimbal->last_angle_change_time = rt_tick_get_millisecond();

    gimbal->calibration_monitor_running = true;

    /* 创建监控线程 */
    gimbal->calibration_monitor_thread = rt_thread_create("cal_monitor",
                                                          calibration_monitor_thread_entry,
                                                          gimbal,
                                                          2048,
                                                          RT_THREAD_PRIORITY_MAX / 2,
                                                          20);

    if (gimbal->calibration_monitor_thread == RT_NULL)
    {
        gimbal->calibration_monitor_running = false;
        LOG_E("Failed to create calibration monitor thread");
        return -RT_ERROR;
    }

    rt_thread_startup(gimbal->calibration_monitor_thread);

    return RT_EOK;
}

/**
 * @brief 停止校准角度监控
 */
rt_err_t laser_gimbal_stop_calibration_monitor(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (!gimbal->calibration_monitor_running)
    {
        return RT_EOK;
    }

    gimbal->calibration_monitor_running = false;

    /* 等待线程结束 */
    if (gimbal->calibration_monitor_thread != RT_NULL)
    {
        rt_thread_delete(gimbal->calibration_monitor_thread);
        gimbal->calibration_monitor_thread = RT_NULL;
    }

    return RT_EOK;
}

/* ======================== 陀螺仪补偿功能实现 ======================== */

/**
 * @brief 启用/禁用陀螺仪补偿功能
 */
rt_err_t laser_gimbal_enable_gyro_compensation(laser_gimbal_t *gimbal, bool enable)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->gyro_compensation_enabled = enable;

    if (enable)
    {
        /* 重置陀螺仪数据 */
        gimbal->last_gyro_yaw = 0.0f;
        gimbal->last_gyro_pitch = 0.0f;
        gimbal->last_gyro_roll = 0.0f;
        gimbal->last_gyro_update_time = rt_tick_get_millisecond();
        
        rt_mutex_release(gimbal->gimbal_mutex);
        
        /* 启动陀螺仪补偿线程 */
        if (laser_gimbal_start_gyro_compensation_thread(gimbal) != RT_EOK)
        {
            gimbal->gyro_compensation_enabled = false;
            LOG_E("Failed to start gyro compensation thread");
            return -RT_ERROR;
        }
        
        LOG_I("Gyro compensation ENABLED - Gain: yaw=%.3f, pitch=%.3f", 
              gimbal->gyro_compensation_gain_yaw, gimbal->gyro_compensation_gain_pitch);
    }
    else
    {
        rt_mutex_release(gimbal->gimbal_mutex);
        
        /* 停止陀螺仪补偿线程 */
        laser_gimbal_stop_gyro_compensation_thread();
        
        LOG_I("Gyro compensation DISABLED");
    }

    return RT_EOK;
}

/**
 * @brief 设置陀螺仪补偿增益
 */
rt_err_t laser_gimbal_set_gyro_compensation_gain(laser_gimbal_t *gimbal, 
                                                float yaw_gain, float pitch_gain)
{
    RT_ASSERT(gimbal != RT_NULL);

    /* 参数验证 */
    if (yaw_gain < 0.0f || yaw_gain > 20.0f || pitch_gain < 0.0f || pitch_gain > 20.0f)
    {
        LOG_E("Invalid gyro compensation gain: yaw=%.3f, pitch=%.3f (range: 0.0-20.0)", 
              yaw_gain, pitch_gain);
        return -RT_EINVAL;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->gyro_compensation_gain_yaw = yaw_gain;
    gimbal->gyro_compensation_gain_pitch = pitch_gain;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Gyro compensation gain set: yaw=%.3f, pitch=%.3f", yaw_gain, pitch_gain);
    return RT_EOK;
}

/**
 * @brief 开始陀螺仪零偏校准
 */
rt_err_t laser_gimbal_start_gyro_calibration(laser_gimbal_t *gimbal, int samples)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (samples <= 0 || samples > 100)
    {
        LOG_E("Invalid calibration samples: %d (range: 1-100)", samples);
        return -RT_EINVAL;
    }

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->gyro_calibration_mode = true;
    gimbal->gyro_calibration_count = 0;
    
    /* 清零校准数据 */
    memset(gimbal->gyro_calibration_samples, 0, sizeof(gimbal->gyro_calibration_samples));

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Gyro calibration started - samples: %d", samples);
    LOG_I("Please keep gimbal STATIONARY during calibration...");
    
    return RT_EOK;
}

/**
 * @brief 停止陀螺仪零偏校准并计算结果
 */
rt_err_t laser_gimbal_finish_gyro_calibration(laser_gimbal_t *gimbal)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (!gimbal->gyro_calibration_mode)
    {
        rt_mutex_release(gimbal->gimbal_mutex);
        LOG_W("Gyro calibration is not active");
        return -RT_ERROR;
    }

    if (gimbal->gyro_calibration_count < 10)
    {
        rt_mutex_release(gimbal->gimbal_mutex);
        LOG_E("Insufficient calibration samples: %d (minimum: 10)", gimbal->gyro_calibration_count);
        return -RT_ERROR;
    }

    /* 计算平均值作为零偏 */
    float sum_yaw = 0.0f, sum_pitch = 0.0f, sum_roll = 0.0f;
    for (int i = 0; i < gimbal->gyro_calibration_count; i++)
    {
        sum_yaw += gimbal->gyro_calibration_samples[0][i];
        sum_pitch += gimbal->gyro_calibration_samples[1][i];
        sum_roll += gimbal->gyro_calibration_samples[2][i];
    }

    gimbal->gyro_offset_yaw = sum_yaw / gimbal->gyro_calibration_count;
    gimbal->gyro_offset_pitch = sum_pitch / gimbal->gyro_calibration_count;
    gimbal->gyro_offset_roll = sum_roll / gimbal->gyro_calibration_count;
    
    gimbal->gyro_calibration_mode = false;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Gyro calibration completed - samples: %d", gimbal->gyro_calibration_count);
    LOG_I("Calculated offsets: yaw=%.3f°/s, pitch=%.3f°/s, roll=%.3f°/s", 
          gimbal->gyro_offset_yaw, gimbal->gyro_offset_pitch, gimbal->gyro_offset_roll);
    
    return RT_EOK;
}

/**
 * @brief 手动设置陀螺仪零偏
 */
rt_err_t laser_gimbal_set_gyro_offset(laser_gimbal_t *gimbal, 
                                     float yaw_offset, float pitch_offset, float roll_offset)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    gimbal->gyro_offset_yaw = yaw_offset;
    gimbal->gyro_offset_pitch = pitch_offset;
    gimbal->gyro_offset_roll = roll_offset;

    rt_mutex_release(gimbal->gimbal_mutex);

    LOG_I("Gyro offsets set: yaw=%.3f°/s, pitch=%.3f°/s, roll=%.3f°/s", 
          yaw_offset, pitch_offset, roll_offset);
    
    return RT_EOK;
}

/**
 * @brief 获取陀螺仪补偿参数
 */
rt_err_t laser_gimbal_get_gyro_compensation_params(laser_gimbal_t *gimbal,
                                                  bool *enabled,
                                                  float *yaw_gain, float *pitch_gain,
                                                  float *yaw_offset, float *pitch_offset, float *roll_offset)
{
    RT_ASSERT(gimbal != RT_NULL);

    if (rt_mutex_take(gimbal->gimbal_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (enabled) *enabled = gimbal->gyro_compensation_enabled;
    if (yaw_gain) *yaw_gain = gimbal->gyro_compensation_gain_yaw;
    if (pitch_gain) *pitch_gain = gimbal->gyro_compensation_gain_pitch;
    if (yaw_offset) *yaw_offset = gimbal->gyro_offset_yaw;
    if (pitch_offset) *pitch_offset = gimbal->gyro_offset_pitch;
    if (roll_offset) *roll_offset = gimbal->gyro_offset_roll;

    rt_mutex_release(gimbal->gimbal_mutex);
    return RT_EOK;
}

/**
 * @brief 更新陀螺仪数据并应用补偿
 */
rt_err_t laser_gimbal_update_gyro_compensation(laser_gimbal_t *gimbal,
                                              float gyro_yaw, float gyro_pitch, float gyro_roll)
{
    RT_ASSERT(gimbal != RT_NULL);

    uint32_t current_time = rt_tick_get_millisecond();

    if (rt_mutex_take(gimbal->gimbal_mutex, 100) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 如果在校准模式，收集校准数据 */
    if (gimbal->gyro_calibration_mode && gimbal->gyro_calibration_count < 100)
    {
        gimbal->gyro_calibration_samples[0][gimbal->gyro_calibration_count] = gyro_yaw;
        gimbal->gyro_calibration_samples[1][gimbal->gyro_calibration_count] = gyro_pitch;
        gimbal->gyro_calibration_samples[2][gimbal->gyro_calibration_count] = gyro_roll;
        gimbal->gyro_calibration_count++;
        
        rt_mutex_release(gimbal->gimbal_mutex);
        return RT_EOK;
    }

    /* 应用零偏校正 */
    float corrected_yaw = gyro_yaw - gimbal->gyro_offset_yaw;
    float corrected_pitch = gyro_pitch - gimbal->gyro_offset_pitch;
    float corrected_roll = gyro_roll - gimbal->gyro_offset_roll;

    /* 更新陀螺仪数据 */
    gimbal->last_gyro_yaw = corrected_yaw;
    gimbal->last_gyro_pitch = corrected_pitch;
    gimbal->last_gyro_roll = corrected_roll;
    gimbal->last_gyro_update_time = current_time;

    /* 如果启用了陀螺仪补偿，计算并应用补偿 */
    if (gimbal->gyro_compensation_enabled && !gimbal->velocity_control_enabled)
    {
        /* 计算时间间隔 */
        float dt = (current_time - gimbal->last_gyro_update_time) / 1000.0f;
        if (dt > 0.1f) dt = 0.01f; // 限制最大时间间隔为100ms
        
        /* 计算补偿角度 (积分陀螺仪数据) */
        float compensation_yaw = -corrected_yaw * gimbal->gyro_compensation_gain_yaw * dt;
        float compensation_pitch = -corrected_pitch * gimbal->gyro_compensation_gain_pitch * dt;
        
        /* 只在陀螺仪数据变化较大时应用补偿 */
        if (fabs(corrected_yaw) > 1.0f || fabs(corrected_pitch) > 1.0f)
        {
            /* 获取当前角度 */
            laser_gimbal_angle_t current_angle;
//            if (laser_gimbal_get_angle(gimbal, &current_angle.yaw, &current_angle.pitch) == RT_EOK)
//            {
//                /* 应用补偿 */
//                laser_gimbal_angle_t compensated_angle;
//                compensated_angle.yaw = current_angle.yaw + compensation_yaw;
//                compensated_angle.pitch = current_angle.pitch + compensation_pitch;
//                
//                /* 角度限制 */
//                compensated_angle.yaw = laser_gimbal_normalize_angle(compensated_angle.yaw);
//                compensated_angle.pitch = MAX(gimbal->angle_limits_min.pitch, 
//                                            MIN(gimbal->angle_limits_max.pitch, compensated_angle.pitch));
//                
//                /* 应用补偿角度 */
//                laser_gimbal_move_to_angle(gimbal, compensated_angle);
//                
					
					laser_gimbal_adjust_angle(gimbal,compensation_yaw,compensation_pitch);
                static uint32_t last_log_time = 0;
                if (current_time - last_log_time > 1000) // 每秒打印一次日志
                {
                    LOG_D("Gyro compensation applied: yaw_comp=%.2f°, pitch_comp=%.2f°, gyro=[%.1f,%.1f]°/s", 
                          compensation_yaw, compensation_pitch, corrected_yaw, corrected_pitch);
                    last_log_time = current_time;
                }
//            }
        }
    }

    rt_mutex_release(gimbal->gimbal_mutex);
    return RT_EOK;
}

/**
 * @brief 陀螺仪补偿线程入口函数
 */
static void laser_gimbal_gyro_compensation_thread_entry(void *parameter)
{
    laser_gimbal_t *gimbal = (laser_gimbal_t *)parameter;
    
    LOG_I("Gyro compensation thread started");
    
    while (g_gyro_compensation_thread_running)
    {
        if (gimbal && gimbal->gyro_compensation_enabled)
        {
            float gyro_x, gyro_y, gyro_z;
            
            /* 从陀螺仪获取数据 */
            if (hipnuc_get_gyro_data(&gyro_x, &gyro_y, &gyro_z) == 0)
            {
                /* 更新陀螺仪补偿 
                 * 注意坐标系映射：
                 * 陀螺仪X轴 -> 云台Yaw轴
                 * 陀螺仪Y轴 -> 云台Pitch轴  
                 * 陀螺仪Z轴 -> 云台Roll轴
                 */
                laser_gimbal_update_gyro_compensation(gimbal, gyro_z, gyro_y, gyro_x);
            }
        }
        
        /* 20ms更新周期，50Hz */
        rt_thread_mdelay(20);
    }
    
    LOG_I("Gyro compensation thread stopped");
}

/**
 * @brief 启动陀螺仪补偿线程
 */
static rt_err_t laser_gimbal_start_gyro_compensation_thread(laser_gimbal_t *gimbal)
{
    if (g_gyro_compensation_thread != RT_NULL)
    {
        return RT_EOK; // 线程已经运行
    }
    
    g_active_gimbal = gimbal;
    g_gyro_compensation_thread_running = true;
    
    g_gyro_compensation_thread = rt_thread_create("gyro_comp",
                                                  laser_gimbal_gyro_compensation_thread_entry,
                                                  gimbal,
                                                  1024,
                                                  5,
                                                  10);
    
    if (g_gyro_compensation_thread == RT_NULL)
    {
        LOG_E("Failed to create gyro compensation thread");
        g_gyro_compensation_thread_running = false;
        return -RT_ERROR;
    }
    
    rt_thread_startup(g_gyro_compensation_thread);
    LOG_I("Gyro compensation thread started successfully");
    
    return RT_EOK;
}

/**
 * @brief 停止陀螺仪补偿线程
 */
static rt_err_t laser_gimbal_stop_gyro_compensation_thread(void)
{
    if (g_gyro_compensation_thread == RT_NULL)
    {
        return RT_EOK; // 线程未运行
    }
    
    g_gyro_compensation_thread_running = false;
    
    /* 等待线程结束 */
    rt_thread_delete(g_gyro_compensation_thread);
    g_gyro_compensation_thread = RT_NULL;
    g_active_gimbal = RT_NULL;
    
    LOG_I("Gyro compensation thread stopped successfully");
    return RT_EOK;
}
