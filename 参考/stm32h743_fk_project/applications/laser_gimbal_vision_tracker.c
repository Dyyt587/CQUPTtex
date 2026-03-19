/**
 * @file laser_gimbal_vision_tracker.c
 * @brief 基于视觉的激光云台追踪控制系统实现
 * @version 1.0
 * @date 2025-07-30
 * @author Your Name
 * @copyright Copyright (c) 2025
 */

#include "laser_gimbal_vision_tracker.h"
#include <rtthread.h>
#include <string.h>
#include <math.h>
#include "apid.h"
#include "hipnuc_dec.h"

#define DBG_TAG "vision_tracker"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* 私有函数声明 */
static rt_err_t vision_pid_init(vision_pid_controller_t *pid, float kp, float ki, float kd);
static float vision_pid_calculate(vision_pid_controller_t *pid, float error, uint32_t current_time);
static rt_err_t vision_tracker_process_target(vision_tracker_t *tracker, float *out_yaw, float *out_pitch);
rt_err_t vision_tracker_process_auto_search(vision_tracker_t *tracker);
static void vision_tracker_thread_entry(void *parameter);
static rt_err_t vision_tracker_apply_control(vision_tracker_t *tracker, float yaw_output, float pitch_output);
static rt_err_t vision_tracker_update_target_history(vision_tracker_t *tracker, vision_target_t *target);
static vision_target_t vision_tracker_filter_target(vision_tracker_t *tracker);

float kp_angle_yaw = 0.5f;
float kp_vel_yaw = 0.5f;
float kp_acc_x_yaw = 268.f;
/* ======================== 核心控制函数实现 ======================== */

/**
 * @brief 初始化视觉追踪器
 */
rt_err_t vision_tracker_init(vision_tracker_t *tracker,
                             laser_gimbal_t *gimbal,
                             uint16_t camera_width,
                             uint16_t camera_height,
                             float fov_h,
                             float fov_v)
{
    RT_ASSERT(tracker != RT_NULL);
    RT_ASSERT(gimbal != RT_NULL);

    LOG_I("Initializing vision tracker...");

    /* 清零结构体 */
    memset(tracker, 0, sizeof(vision_tracker_t));

    /* 基本参数初始化 */
    tracker->gimbal = gimbal;
    tracker->mode = VISION_TRACK_MODE_DISABLED;
    tracker->state = VISION_TRACK_STATE_IDLE;
    tracker->enabled = false;

    /* 摄像头参数初始化 */
    tracker->camera.width = camera_width;
    tracker->camera.height = camera_height;
    tracker->camera.fov_horizontal = fov_h;
    tracker->camera.fov_vertical = fov_v;
    tracker->camera.center_x = camera_width / 2.0f;
    tracker->camera.center_y = camera_height / 2.0f;

    /* PID控制器初始化 */
    vision_pid_init(&tracker->pid_yaw,
                    VISION_PID_DEFAULT_KP_YAW,
                    VISION_PID_DEFAULT_KI_YAW,
                    VISION_PID_DEFAULT_KD_YAW);

    vision_pid_init(&tracker->pid_pitch,
                    VISION_PID_DEFAULT_KP_PITCH,
                    VISION_PID_DEFAULT_KI_PITCH,
                    VISION_PID_DEFAULT_KD_PITCH);

    /* 追踪参数初始化 */
    tracker->deadzone_pixels = VISION_PID_DEADZONE;
    tracker->max_error_threshold = VISION_TRACKER_MAX_ERROR_THRESHOLD;
    tracker->target_lost_timeout = VISION_TRACKER_TARGET_LOST_TIMEOUT;

    /* 偏移参数初始化 */
    tracker->offset_x = 0.0f;
    tracker->offset_y = 0.0f;
    tracker->offset_enabled = false;

    /* 自动搜索参数初始化 */
    tracker->confidence_threshold = 0.5f; // 可信度阈值
    tracker->auto_search_enabled = true;  // 默认启用自动搜索
    tracker->search_yaw_speed = 60.0f;    // 搜索速度30度/秒
    tracker->search_pitch_angle = 0.0f;   // pitch锁定为0度
    tracker->search_start_time = 0;
    tracker->search_start_yaw = 0.0f;
    tracker->search_cooldown_time = 500; // 默认冷却时间0.5秒
    tracker->last_search_end_time = 0;
    tracker->max_search_time = 12000; // 最大搜索时间12秒(360度/30度每秒)
    tracker->search_completed_full_cycle = false;

    /* 陀螺仪补偿参数初始化 */
    tracker->gyro_compensation_enabled = true;    // 默认启用陀螺仪补偿
    tracker->gyro_compensation_gain_yaw = 0.3f;   // Yaw轴补偿增益
    tracker->gyro_compensation_gain_pitch = 0.3f; // Pitch轴补偿增益
    tracker->gyro_motion_threshold = 2.0f;        // 运动阈值(度/秒)
    tracker->gyro_last_log_time = 0;              // 上次日志时间

    /* 创建互斥锁 */
    tracker->tracker_mutex = rt_mutex_create("vis_track", RT_IPC_FLAG_PRIO);
    if (tracker->tracker_mutex == RT_NULL)
    {
        LOG_E("Failed to create tracker mutex");
        return -RT_ERROR;
    }

    /* 创建追踪线程 */
    tracker->track_thread = rt_thread_create("vis_track",
                                             vision_tracker_thread_entry,
                                             tracker,
                                             2048,
                                             10,
                                             20);
    if (tracker->track_thread == RT_NULL)
    {
        LOG_E("Failed to create tracking thread");
        rt_mutex_delete(tracker->tracker_mutex);
        return -RT_ERROR;
    }

    tracker->thread_running = true;
    rt_thread_startup(tracker->track_thread);

    LOG_I("Vision tracker initialized: %dx%d, FOV: %.1f°x%.1f°",
          camera_width, camera_height, fov_h, fov_v);

    return RT_EOK;
}

/**
 * @brief 反初始化视觉追踪器
 */
rt_err_t vision_tracker_deinit(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    LOG_I("Deinitializing vision tracker...");

    /* 停止追踪 */
    vision_tracker_enable(tracker, false);

    /* 停止线程 */
    if (tracker->track_thread != RT_NULL)
    {
        tracker->thread_running = false;
        rt_thread_delete(tracker->track_thread);
        tracker->track_thread = RT_NULL;
    }

    /* 删除互斥锁 */
    if (tracker->tracker_mutex != RT_NULL)
    {
        rt_mutex_delete(tracker->tracker_mutex);
        tracker->tracker_mutex = RT_NULL;
    }

    LOG_I("Vision tracker deinitialized");
    return RT_EOK;
}

/**
 * @brief 启用/禁用视觉追踪
 */
rt_err_t vision_tracker_enable(vision_tracker_t *tracker, bool enable)
{
    RT_ASSERT(tracker != RT_NULL);

    // if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    // {
    //     return -RT_ERROR;
    // }

    tracker->enabled = enable;

    if (enable)
    {
        tracker->state = VISION_TRACK_STATE_SEARCHING;
        vision_tracker_reset_pid(tracker);
        LOG_I("Vision tracking enabled");
        rt_thread_mdelay(1800);
        // rt_pin_write(GET_PIN(D, 13), PIN_LOW);//记得加回去
    }
    else
    {
        tracker->state = VISION_TRACK_STATE_IDLE;
        tracker->mode = VISION_TRACK_MODE_DISABLED;
        LOG_I("Vision tracking disabled");
    }

    // rt_mutex_release(tracker->tracker_mutex);
    return RT_EOK;
}

/**
 * @brief 设置追踪模式
 */
rt_err_t vision_tracker_set_mode(vision_tracker_t *tracker, vision_track_mode_t mode)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    tracker->mode = mode;

    /* 根据模式调整状态 */
    if (mode == VISION_TRACK_MODE_DISABLED)
    {
        tracker->enabled = false;
        tracker->state = VISION_TRACK_STATE_IDLE;
    }
    else if (tracker->enabled)
    {
        tracker->state = VISION_TRACK_STATE_SEARCHING;
        vision_tracker_reset_pid(tracker);
    }

    rt_mutex_release(tracker->tracker_mutex);

    const char *mode_names[] = {"DISABLED", "MANUAL", "CONTINUOUS", "LOCK_ON"};
    LOG_I("Vision tracking mode set to: %s", mode_names[mode]);

    return RT_EOK;
}

/* ======================== 目标输入函数实现 ======================== */

/**
 * @brief 输入目标像素坐标
 */
rt_err_t vision_tracker_input_target(vision_tracker_t *tracker,
                                     float pixel_x,
                                     float pixel_y,
                                     float confidence)
{
    RT_ASSERT(tracker != RT_NULL);

    /* 参数验证 */
    if (pixel_x < 0 || pixel_x >= tracker->camera.width ||
        pixel_y < 0 || pixel_y >= tracker->camera.height)
    {
        LOG_W("Pixel coordinates out of range: (%.1f, %.1f)", pixel_x, pixel_y);
        return -RT_EINVAL;
    }

    if (confidence < 0.0f || confidence > 1.0f)
    {
        LOG_W("Invalid confidence value: %.2f", confidence);
        confidence = MAX(0.0f, MIN(1.0f, confidence));
    }

    /* 构造目标信息 */
    vision_target_t target;
    target.pixel_coord.x = pixel_x;
    target.pixel_coord.y = pixel_y;
    target.pixel_coord.valid = true;
    target.pixel_coord.timestamp = rt_tick_get_millisecond();
    target.confidence = confidence;
    target.size = 0.0f;  // 未知大小
    target.track_id = 0; // 默认ID

    return vision_tracker_input_target_info(tracker, &target);
}

/**
 * @brief 输入完整目标信息
 */
rt_err_t vision_tracker_input_target_info(vision_tracker_t *tracker,
                                          vision_target_t *target)
{
    RT_ASSERT(tracker != RT_NULL);
    RT_ASSERT(target != RT_NULL);

    if (!tracker->enabled)
    {
        return -RT_ERROR;
    }

    //    if (rt_mutex_take(tracker->tracker_mutex, 100) != RT_EOK)
    //    {
    //        LOG_W("Failed to take mutex for target input");
    //        return -RT_ERROR;
    //    }

    /* 更新目标历史 */
    vision_tracker_update_target_history(tracker, target);

    /* 滤波处理目标 */
    tracker->current_target = vision_tracker_filter_target(tracker);

    /* 更新状态 */
    tracker->last_target_time = rt_tick_get_millisecond();

    /* 检查可信度是否需要触发自动搜索 */

    static int cc_cnt = 0;

    if (tracker->auto_search_enabled &&
        target->confidence < tracker->confidence_threshold &&
        tracker->state != VISION_TRACK_STATE_AUTO_SEARCHING)
    {
        cc_cnt++;
    }
    else
    {
        cc_cnt = 0;
    }

    if (cc_cnt > 100)
    {
        /* 检查冷却时间 */
        uint32_t current_time = rt_tick_get_millisecond();
        if (tracker->last_search_end_time == 0 ||
            (current_time - tracker->last_search_end_time) >= tracker->search_cooldown_time)
        {
            LOG_I("Low confidence detected (%.2f < %.2f), starting auto search",
                  target->confidence, tracker->confidence_threshold);
            // vision_tracker_start_auto_search(tracker);
        }
        else
        {
            uint32_t remaining_cooldown = tracker->search_cooldown_time - (current_time - tracker->last_search_end_time);
            LOG_D("Auto search in cooldown, remaining: %dms", remaining_cooldown);
        }
        return RT_EOK;
    }

    /* 如果在自动搜索中且可信度恢复，则停止搜索 */
    if (tracker->state == VISION_TRACK_STATE_AUTO_SEARCHING &&
        target->confidence >= tracker->confidence_threshold)
    {
        LOG_I("High confidence detected during search (%.2f >= %.2f), stopping auto search",
              target->confidence, tracker->confidence_threshold);
        vision_tracker_stop_auto_search(tracker);
        tracker->state = VISION_TRACK_STATE_TRACKING;
    }

    if (tracker->state == VISION_TRACK_STATE_SEARCHING)
    {
        tracker->state = VISION_TRACK_STATE_TRACKING;
        if (tracker->on_target_acquired)
        {
            tracker->on_target_acquired(tracker->current_target);
        }
        //        LOG_I("Target acquired at (%.1f, %.1f), confidence: %.2f",
        //              target->pixel_coord.x, target->pixel_coord.y, target->confidence);
    }

    tracker->track_count++;

    // rt_mutex_release(tracker->tracker_mutex);
    return RT_EOK;
}

/**
 * @brief 标记目标丢失
 */
rt_err_t vision_tracker_target_lost(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, 100) != RT_EOK)
    {
        return -RT_ERROR;
    }

    tracker->current_target.pixel_coord.valid = false;
    tracker->state = VISION_TRACK_STATE_TARGET_LOST;
    tracker->lost_count++;

    if (tracker->on_target_lost)
    {
        tracker->on_target_lost();
    }

    LOG_W("Target lost");

    rt_mutex_release(tracker->tracker_mutex);
    return RT_EOK;
}

/* ======================== PID参数配置函数实现 ======================== */

/**
 * @brief 设置Yaw轴PID参数
 */
rt_err_t vision_tracker_set_yaw_pid(vision_tracker_t *tracker,
                                    float kp, float ki, float kd)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 设置APID参数 */
    tracker->pid_yaw.controller.parameter.kp = kp;
    tracker->pid_yaw.controller.parameter.ki = ki;
    tracker->pid_yaw.controller.parameter.kd = kd;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Yaw PID parameters set: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki, kd);
    return RT_EOK;
}

/**
 * @brief 设置Pitch轴PID参数
 */
rt_err_t vision_tracker_set_pitch_pid(vision_tracker_t *tracker,
                                      float kp, float ki, float kd)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 设置APID参数 */
    tracker->pid_pitch.controller.parameter.kp = kp;
    tracker->pid_pitch.controller.parameter.ki = ki;
    tracker->pid_pitch.controller.parameter.kd = kd;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Pitch PID parameters set: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki, kd);
    return RT_EOK;
}

/**
 * @brief 设置PID输出限制
 */
rt_err_t vision_tracker_set_pid_limits(vision_tracker_t *tracker,
                                       float max_output,
                                       float max_integral)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* 设置APID输出限制和积分限制 */
    APID_Set_Out_Limit(&tracker->pid_yaw.controller, max_output);
    APID_Set_Integral_Limit(&tracker->pid_yaw.controller, max_integral);
    APID_Set_Out_Limit(&tracker->pid_pitch.controller, max_output);
    APID_Set_Integral_Limit(&tracker->pid_pitch.controller, max_integral);

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("PID limits set: max_output=%.1f°/s, max_integral=%.1f", max_output, max_integral);
    return RT_EOK;
}

/* ======================== 追踪参数配置函数实现 ======================== */

/**
 * @brief 设置追踪参数
 */
rt_err_t vision_tracker_set_track_params(vision_tracker_t *tracker,
                                         float deadzone,
                                         float max_error,
                                         uint32_t lost_timeout)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    tracker->deadzone_pixels = deadzone;
    tracker->max_error_threshold = max_error;
    tracker->target_lost_timeout = lost_timeout;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Tracking params set: deadzone=%.1fpx, max_error=%.1fpx, timeout=%dms",
          deadzone, max_error, lost_timeout);
    return RT_EOK;
}

/**
 * @brief 设置目标偏移量
 */
rt_err_t vision_tracker_set_offset(vision_tracker_t *tracker,
                                   float offset_x,
                                   float offset_y)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    tracker->offset_x = offset_x;
    tracker->offset_y = offset_y;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Target offset set: x=%.1fpx, y=%.1fpx", offset_x, offset_y);
    return RT_EOK;
}

/**
 * @brief 启用/禁用目标偏移
 */
rt_err_t vision_tracker_enable_offset(vision_tracker_t *tracker, bool enable)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    tracker->offset_enabled = enable;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Target offset %s", enable ? "ENABLED" : "DISABLED");
    return RT_EOK;
}

/**
 * @brief 获取当前偏移设置
 */
rt_err_t vision_tracker_get_offset(vision_tracker_t *tracker,
                                   float *offset_x,
                                   float *offset_y,
                                   bool *enabled)
{
    RT_ASSERT(tracker != RT_NULL);
    RT_ASSERT(offset_x != RT_NULL);
    RT_ASSERT(offset_y != RT_NULL);
    RT_ASSERT(enabled != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    *offset_x = tracker->offset_x;
    *offset_y = tracker->offset_y;
    *enabled = tracker->offset_enabled;

    rt_mutex_release(tracker->tracker_mutex);

    return RT_EOK;
}

/* ======================== 自动搜索功能实现 ======================== */

/**
 * @brief 设置自动搜索参数
 */
rt_err_t vision_tracker_set_auto_search_params(vision_tracker_t *tracker,
                                               float confidence_threshold,
                                               float search_speed,
                                               float pitch_angle,
                                               uint32_t max_search_time,
                                               uint32_t cooldown_time)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    tracker->confidence_threshold = confidence_threshold;
    tracker->search_yaw_speed = search_speed;
    tracker->search_pitch_angle = pitch_angle;
    tracker->max_search_time = max_search_time;
    tracker->search_cooldown_time = cooldown_time;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Auto search params set: threshold=%.2f, speed=%.1f°/s, pitch=%.1f°, max_time=%dms, cooldown=%dms",
          confidence_threshold, search_speed, pitch_angle, max_search_time, cooldown_time);
    return RT_EOK;
}

/**
 * @brief 启用/禁用自动搜索功能
 */
rt_err_t vision_tracker_enable_auto_search(vision_tracker_t *tracker, bool enable)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    tracker->auto_search_enabled = enable;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Auto search %s", enable ? "ENABLED" : "DISABLED");
    return RT_EOK;
}

/**
 * @brief 获取自动搜索设置
 */
rt_err_t vision_tracker_get_auto_search_params(vision_tracker_t *tracker,
                                               float *confidence_threshold,
                                               float *search_speed,
                                               float *pitch_angle,
                                               uint32_t *cooldown_time,
                                               bool *enabled)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (confidence_threshold)
        *confidence_threshold = tracker->confidence_threshold;
    if (search_speed)
        *search_speed = tracker->search_yaw_speed;
    if (pitch_angle)
        *pitch_angle = tracker->search_pitch_angle;
    if (cooldown_time)
        *cooldown_time = tracker->search_cooldown_time;
    if (enabled)
        *enabled = tracker->auto_search_enabled;

    rt_mutex_release(tracker->tracker_mutex);

    return RT_EOK;
}

/**
 * @brief 手动触发自动搜索
 */
rt_err_t vision_tracker_start_auto_search(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    if (!tracker->enabled || !tracker->auto_search_enabled)
    {
        return -RT_ERROR;
    }

    /* 记录搜索开始信息 */
    tracker->search_start_time = rt_tick_get_millisecond();
    tracker->search_completed_full_cycle = false;
    tracker->state = VISION_TRACK_STATE_AUTO_SEARCHING;

    //    /* 获取当前yaw角度作为搜索起始点 */
    //    laser_gimbal_get_angle(tracker->gimbal, &tracker->search_start_yaw, RT_NULL);
    // tracker->search_start_yaw = tracker->history_index > 0 ?
    //                             tracker->target_history[tracker->history_index - 1].pixel_coord.x :
    //                             0.0f; // 如果没有历史数据，则默认为0

    // tracker->search_pitch_angle = tracker->history_index > 0 ?
    //                               tracker->target_history[tracker->history_index - 1].pixel_coord.y :
    //                               0.0f; // 如果没有历史数据，则默认为0

    tracker->search_start_yaw = 0; // 如果没有历史数据，则默认为0

    tracker->search_pitch_angle = 0; // 如果没有历史数据，则默认为0
    /* 设置pitch轴为搜索位置 */
    // laser_gimbal_set_pitch(tracker->gimbal, tracker->search_pitch_angle);

    LOG_I("Auto search started from yaw=%.1f°, pitch=%.1f°",
          tracker->search_start_yaw, tracker->search_pitch_angle);

    return RT_EOK;
}

/**
 * @brief 停止自动搜索
 */
rt_err_t vision_tracker_stop_auto_search(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    if (tracker->state == VISION_TRACK_STATE_AUTO_SEARCHING)
    {
        tracker->state = VISION_TRACK_STATE_SEARCHING;
        tracker->last_search_end_time = rt_tick_get_millisecond(); // 记录搜索结束时间
        LOG_I("Auto search stopped, cooldown started for %dms", tracker->search_cooldown_time);
    }

    return RT_EOK;
}

/* ======================== 状态查询函数实现 ======================== */

/**
 * @brief 获取追踪状态
 */
vision_track_state_t vision_tracker_get_state(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);
    return tracker->state;
}

/**
 * @brief 获取当前目标信息
 */
rt_err_t vision_tracker_get_current_target(vision_tracker_t *tracker,
                                           vision_target_t *target)
{
    RT_ASSERT(tracker != RT_NULL);
    RT_ASSERT(target != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, 100) != RT_EOK)
    {
        return -RT_ERROR;
    }

    *target = tracker->current_target;

    rt_mutex_release(tracker->tracker_mutex);
    return RT_EOK;
}

/**
 * @brief 获取PID误差信息
 */
rt_err_t vision_tracker_get_pid_errors(vision_tracker_t *tracker,
                                       float *yaw_error,
                                       float *pitch_error)
{
    RT_ASSERT(tracker != RT_NULL);
    RT_ASSERT(yaw_error != RT_NULL);
    RT_ASSERT(pitch_error != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, 100) != RT_EOK)
    {
        return -RT_ERROR;
    }

    *yaw_error = tracker->pid_yaw.error;
    *pitch_error = tracker->pid_pitch.error;

    rt_mutex_release(tracker->tracker_mutex);
    return RT_EOK;
}

/**
 * @brief 获取统计信息
 */
rt_err_t vision_tracker_get_statistics(vision_tracker_t *tracker,
                                       uint32_t *track_count,
                                       uint32_t *success_count,
                                       uint32_t *lost_count)
{
    RT_ASSERT(tracker != RT_NULL);

    if (track_count)
        *track_count = tracker->track_count;
    if (success_count)
        *success_count = tracker->success_count;
    if (lost_count)
        *lost_count = tracker->lost_count;

    return RT_EOK;
}

/* ======================== 实用工具函数实现 ======================== */

/**
 * @brief 像素坐标转角度误差
 */
rt_err_t vision_tracker_pixel_to_angle_error(vision_tracker_t *tracker,
                                             float pixel_x,
                                             float pixel_y,
                                             float *yaw_error,
                                             float *pitch_error)
{
    RT_ASSERT(tracker != RT_NULL);
    RT_ASSERT(yaw_error != RT_NULL);
    RT_ASSERT(pitch_error != RT_NULL);

    /* 应用用户设置的偏移量 */
    float adjusted_pixel_x = pixel_x;
    float adjusted_pixel_y = pixel_y;

    if (tracker->offset_enabled)
    {
        adjusted_pixel_x += tracker->offset_x;
        adjusted_pixel_y += tracker->offset_y;
    }

    /* 计算像素偏移 */
    float pixel_offset_x = adjusted_pixel_x - tracker->camera.center_x;
    float pixel_offset_y = adjusted_pixel_y - tracker->camera.center_y;

    /* 转换为角度误差 */
    *yaw_error = (pixel_offset_x / tracker->camera.center_x) * (tracker->camera.fov_horizontal / 2.0f);
    *pitch_error = (pixel_offset_y / tracker->camera.center_y) * (tracker->camera.fov_vertical / 2.0f);

    return RT_EOK;
}

/**
 * @brief 重置PID控制器
 */
rt_err_t vision_tracker_reset_pid(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    /* 重置APID控制器 */
    APID_Reset(&tracker->pid_yaw.controller);
    APID_Reset(&tracker->pid_pitch.controller);

    /* 重新初始化 */
    APID_Init(&tracker->pid_yaw.controller, 0, // APID_POSITION = 0
              VISION_PID_DEFAULT_KP_YAW,
              VISION_PID_DEFAULT_KI_YAW,
              VISION_PID_DEFAULT_KD_YAW);

    /* 重新初始化 */
    APID_Init(&tracker->pid_pitch.controller, APID_POSITION, // APID_POSITION = 0
              VISION_PID_DEFAULT_KP_PITCH,
              VISION_PID_DEFAULT_KI_PITCH,
              VISION_PID_DEFAULT_KD_PITCH);

    /* 设置限制 */
    APID_Set_Out_Limit(&tracker->pid_yaw.controller, VISION_PID_MAX_OUTPUT);
    APID_Set_Out_Limit(&tracker->pid_pitch.controller, VISION_PID_MAX_OUTPUT);

    tracker->pid_yaw.error = 0.0f;
    tracker->pid_yaw.output = 0.0f;
    tracker->pid_yaw.last_time = rt_tick_get_millisecond();

    tracker->pid_pitch.error = 0.0f;
    tracker->pid_pitch.output = 0.0f;
    tracker->pid_pitch.last_time = rt_tick_get_millisecond();

    return RT_EOK;
}

/**
 * @brief 显示追踪器状态信息
 */
void vision_tracker_show_status(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    const char *state_names[] = {"IDLE", "SEARCHING", "AUTO_SEARCHING", "TRACKING", "LOCKED", "TARGET_LOST", "ERROR"};
    const char *mode_names[] = {"DISABLED", "MANUAL", "CONTINUOUS", "LOCK_ON"};

    LOG_I("=== Vision Tracker Status ===");
    LOG_I("State: %s, Mode: %s, Enabled: %s",
          state_names[tracker->state],
          mode_names[tracker->mode],
          tracker->enabled ? "Yes" : "No");

    LOG_I("Camera: %dx%d, FOV: %.1f°x%.1f°",
          tracker->camera.width, tracker->camera.height,
          tracker->camera.fov_horizontal, tracker->camera.fov_vertical);

    if (tracker->current_target.pixel_coord.valid)
    {
        LOG_I("Current target: (%.1f, %.1f), confidence: %.2f",
              tracker->current_target.pixel_coord.x,
              tracker->current_target.pixel_coord.y,
              tracker->current_target.confidence);
    }
    else
    {
        LOG_I("Current target: INVALID");
    }

    LOG_I("PID Yaw: Kp=%.3f, Ki=%.3f, Kd=%.3f, Error=%.2f°",
          tracker->pid_yaw.controller.parameter.kp, tracker->pid_yaw.controller.parameter.ki, tracker->pid_yaw.controller.parameter.kd, tracker->pid_yaw.error);

    LOG_I("PID Pitch: Kp=%.3f, Ki=%.3f, Kd=%.3f, Error=%.2f°",
          tracker->pid_pitch.controller.parameter.kp, tracker->pid_pitch.controller.parameter.ki, tracker->pid_pitch.controller.parameter.kd, tracker->pid_pitch.error);

    LOG_I("Target Offset: %s, X=%.1fpx, Y=%.1fpx",
          tracker->offset_enabled ? "ENABLED" : "DISABLED",
          tracker->offset_x, tracker->offset_y);

    LOG_I("Auto Search: %s, Threshold=%.2f, Speed=%.1f°/s, Pitch=%.1f°, Cooldown=%dms",
          tracker->auto_search_enabled ? "ENABLED" : "DISABLED",
          tracker->confidence_threshold, tracker->search_yaw_speed, tracker->search_pitch_angle, tracker->search_cooldown_time);

    /* 显示冷却状态 */
    if (tracker->last_search_end_time > 0)
    {
        uint32_t current_time = rt_tick_get_millisecond();
        uint32_t time_since_last_search = current_time - tracker->last_search_end_time;
        if (time_since_last_search < tracker->search_cooldown_time)
        {
            uint32_t remaining_cooldown = tracker->search_cooldown_time - time_since_last_search;
            LOG_I("Search Cooldown: Active, remaining %dms", remaining_cooldown);
        }
        else
        {
            LOG_I("Search Cooldown: Ready");
        }
    }
    else
    {
        LOG_I("Search Cooldown: Ready (never searched)");
    }

    LOG_I("Statistics: Track=%d, Success=%d, Lost=%d",
          tracker->track_count, tracker->success_count, tracker->lost_count);
}

/* ======================== 私有函数实现 ======================== */

/**
 * @brief 初始化PID控制器
 */
static rt_err_t vision_pid_init(vision_pid_controller_t *pid, float kp, float ki, float kd)
{
    RT_ASSERT(pid != RT_NULL);

    memset(pid, 0, sizeof(vision_pid_controller_t));

    /* 初始化APID控制器 */
    // APID_Reset(&pid->controller);
    APID_Init(&pid->controller, APID_POSITION, kp, ki, kd);
    // pid->controller.parameter.kp = kp;
    // pid->controller.parameter.ki = ki;
    // pid->controller.parameter.kd = kd;
    APID_Set_Out_Limit(&pid->controller, VISION_PID_MAX_OUTPUT);
    APID_Set_Bias_Limit(&pid->controller, VISION_PID_BISO_MAX);

    pid->last_time = rt_tick_get_millisecond();

    return RT_EOK;
}

/**
 * @brief 计算PID输出
 */
static float vision_pid_calculate(vision_pid_controller_t *pid, float error, uint32_t current_time)
{
    RT_ASSERT(pid != RT_NULL);

    /* 更新误差 */
    pid->error = -error;

    pid->controller.parameter.present = error;

    //	rt_enter_critical();
    APID_Hander(&pid->controller, 0.01f);
    // rt_critical_level();
    /* 使用APID控制器计算输出 */
    // pid->output = APID_Realize(&pid->controller, 0.0f, error);
    pid->output = APID_Get_Out(&pid->controller);

    extern vision_tracker_t g_vision_tracker;

    // if(pid == &g_vision_tracker.pid_pitch) LOG_RAW("PID:%f,%f,%f\n", pid->controller.parameter.target, pid->controller.parameter.present, pid->controller.parameter.out);
    if (pid == &g_vision_tracker.pid_yaw)
        LOG_RAW("PID:%f,%f,%f\n", pid->controller.parameter.target, pid->controller.parameter.present, pid->controller.parameter.out);
    pid->last_time = current_time;

    return pid->output;
}

/**
 * @brief 处理目标并计算控制输出
 */
static rt_err_t vision_tracker_process_target(vision_tracker_t *tracker, float *out_yaw, float *out_pitch)
{

    rt_err_t result = RT_EOK;
    RT_ASSERT(tracker != RT_NULL);
    float yaw_output = 0;
    float pitch_output = 0;
    if (!tracker->enabled || !tracker->current_target.pixel_coord.valid)
    {
        result = -RT_ERROR;
        // return result;
        goto FIX;
    }

    /* 检查目标丢失超时 */
    uint32_t current_time = rt_tick_get_millisecond();
    if ((current_time - tracker->last_target_time) > tracker->target_lost_timeout)
    {
        tracker->state = VISION_TRACK_STATE_TARGET_LOST;
        tracker->current_target.pixel_coord.valid = false;
        if (tracker->on_target_lost)
        {
            tracker->on_target_lost();
        }
        result = -RT_ERROR;
        goto FIX;

        //  return result;
    }

    /* 将像素坐标转换为角度误差 */
    float yaw_error_deg, pitch_error_deg;
    vision_tracker_pixel_to_angle_error(tracker,
                                        tracker->current_target.pixel_coord.x,
                                        tracker->current_target.pixel_coord.y,
                                        &yaw_error_deg,
                                        &pitch_error_deg);

    /* 死区处理 */
    float pixel_distance = powf(tracker->current_target.pixel_coord.x - tracker->camera.center_x, 2) +
                           powf(tracker->current_target.pixel_coord.y - tracker->camera.center_y, 2);

    if (pixel_distance < tracker->deadzone_pixels * tracker->deadzone_pixels)
    {
        /* 在死区内，认为已锁定目标 */
        if (tracker->state != VISION_TRACK_STATE_LOCKED)
        {
            tracker->state = VISION_TRACK_STATE_LOCKED;
            tracker->success_count++;
            LOG_I("Target locked (within deadzone)");
            extern void buzzer_on(int argc, char **argv);
            buzzer_on(0, NULL);

            vision_tracker_apply_control(tracker, 0, 0);
        }
        else
        {
            // extern void buzzer_off(int argc, char **argv);
            // buzzer_off(0, NULL);
        }
        goto FIX;

        // return RT_EOK; // 不需要控制
    }
    /* 计算PID输出 */

    yaw_output = vision_pid_calculate(&tracker->pid_yaw, yaw_error_deg, current_time);
    pitch_output = vision_pid_calculate(&tracker->pid_pitch, pitch_error_deg, current_time);

    // if (result != RT_EOK)
    // {
    //     return result;
    // }
FIX:
    /* 应用控制输出 */
   // result = vision_tracker_apply_control(tracker, yaw_output, pitch_output);
    *out_yaw = yaw_output; // 返回yaw输出

    *out_pitch = pitch_output; // 返回pitch输出
    if (result == RT_EOK)
    {
        tracker->state = VISION_TRACK_STATE_TRACKING;
    }

    return result;
}

/**
 * @brief 处理自动搜索逻辑
 */

rt_err_t vision_tracker_process_auto_search(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    uint32_t current_time = rt_tick_get_millisecond();
    uint32_t search_duration = current_time - tracker->search_start_time;

    /* 检查搜索超时 */
    if (search_duration > tracker->max_search_time)
    {
        LOG_W("Auto search timeout after %d ms", search_duration);
        tracker->state = VISION_TRACK_STATE_TARGET_LOST;
        tracker->last_search_end_time = current_time; // 记录搜索结束时间
        return -RT_ERROR;
    }
    /* 固定角度搜索实现 */
    static const float search_angles[] = {0.0f, -45.0f, -90.0f, -135.0f, 180.0f, 135.0f, 90.0f, 45.0f}; // 以45度为间隔
    static const int num_angles = sizeof(search_angles) / sizeof(search_angles[0]);
    static int current_angle_index = 0;

    rt_err_t result = 0;

    // 每隔一段时间切换到下一个角度
    static uint32_t last_switch_time = 0;
    // uint32_t interval = tracker->max_search_time / num_angles; // 平均分配时间
    uint32_t interval = 400; // 平均分配时间

    if (last_switch_time == 0 || (current_time - last_switch_time) > interval)
    {
        last_switch_time = current_time;
        current_angle_index = (current_angle_index + 1) % num_angles;
        LOG_I("Auto search: switching to angle %.1f°", search_angles[current_angle_index]);
    }

    // 控制云台到当前搜索角度
    result = laser_gimbal_set_yaw(tracker->gimbal, search_angles[current_angle_index]);
    // result = laser_gimbal_set_angle(tracker->gimbal, search_angles[current_angle_index], tracker->search_pitch_angle);

    return result;
}

/**
 * @brief 应用控制输出到云台
 */
static rt_err_t vision_tracker_apply_control(vision_tracker_t *tracker, float yaw_output, float pitch_output)
{
    RT_ASSERT(tracker != RT_NULL);

    /* 根据追踪模式决定控制方式 */
    switch (tracker->mode)
    {
    case VISION_TRACK_MODE_MANUAL:
        /* 手动模式：只做一次调整 */
        // return laser_gimbal_adjust_angle(tracker->gimbal, yaw_output, pitch_output);
        return laser_gimbal_set_angular_velocity(tracker->gimbal, yaw_output, pitch_output);

    case VISION_TRACK_MODE_CONTINUOUS:
        /* 连续模式：直接调整角度 */
        return laser_gimbal_set_angular_velocity(tracker->gimbal, yaw_output, pitch_output);

    case VISION_TRACK_MODE_LOCK_ON:
        /* 锁定模式：高精度调整 */
        return laser_gimbal_set_angular_velocity(tracker->gimbal, yaw_output, pitch_output);

    default:
        return -RT_ERROR;
    }
}

/**
 * @brief 更新目标历史
 */
static rt_err_t vision_tracker_update_target_history(vision_tracker_t *tracker, vision_target_t *target)
{
    RT_ASSERT(tracker != RT_NULL);
    RT_ASSERT(target != RT_NULL);

    tracker->target_history[tracker->history_index] = *target;
    tracker->history_index = (tracker->history_index + 1) % 5;

    if (tracker->history_count < 5)
    {
        tracker->history_count++;
    }

    return RT_EOK;
}

/**
 * @brief 滤波目标信息
 */
static vision_target_t vision_tracker_filter_target(vision_tracker_t *tracker)
{
    RT_ASSERT(tracker != RT_NULL);

    if (tracker->history_count == 0)
    {
        vision_target_t empty_target = {0};
        return empty_target;
    }

    /* 简单移动平均滤波 */
    float sum_x = 0.0f, sum_y = 0.0f, sum_conf = 0.0f;
    int valid_count = 0;

    for (int i = 0; i < tracker->history_count; i++)
    {
        if (tracker->target_history[i].pixel_coord.valid)
        {
            sum_x += tracker->target_history[i].pixel_coord.x;
            sum_y += tracker->target_history[i].pixel_coord.y;
            sum_conf += tracker->target_history[i].confidence;
            valid_count++;
        }
    }

    vision_target_t filtered_target = {0};
    if (valid_count > 0)
    {
        filtered_target.pixel_coord.x = sum_x / valid_count;
        filtered_target.pixel_coord.y = sum_y / valid_count;
        filtered_target.pixel_coord.valid = true;
        filtered_target.pixel_coord.timestamp = rt_tick_get_millisecond();
        filtered_target.confidence = sum_conf / valid_count;
    }

    return filtered_target;
}

/**
 * @brief 追踪线程入口函数
 */

static void vision_tracker_thread_entry(void *parameter)
{
    vision_tracker_t *tracker = (vision_tracker_t *)parameter;
    float fix_yaw;
    extern int hipnuc_get_gyro_data(float *gyro_x, float *gyro_y, float *gyro_z);
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float yaw = 0;
    float adj_x_yaw=0;


    float yaw_out = 0;
    float pitch_out = 0;
    LOG_I("Vision tracking thread started");

    while (tracker->thread_running)
    {
        yaw_out = 0;
        pitch_out = 0;
        if (tracker->enabled)
        {
            if (rt_mutex_take(tracker->tracker_mutex, 10) == RT_EOK)
            {

                /* 添加陀螺仪补偿 */

                // {
                //     /* HiPNUC IMU角度+角速度融合补偿 - 针对突然加速优化 */
                // extern hipnuc_raw_t hipnuc_raw; // 原始数据结构体
                // static float last_yaw = 0;
                // static float last_pitch = 0;
                // static float last_gyro_yaw = 0;
                // static float last_gyro_pitch = 0;
                // static int init_cnt = 0;
                // static uint32_t last_imu_time = 0;
                // static float gyro_yaw_integral = 0;
                // static float gyro_pitch_integral = 0;

                // uint32_t current_imu_time = rt_tick_get_millisecond();
                // float dt = (current_imu_time - last_imu_time) / 1000.0f; // 转换为秒

                // if (init_cnt++ < 100)
                // {
                //     last_yaw = hipnuc_raw.hi91.yaw;
                //     last_pitch = hipnuc_raw.hi91.pitch;
                //     last_gyro_yaw = hipnuc_raw.hi91.gyr[2];
                //     last_gyro_pitch = hipnuc_raw.hi91.gyr[1];
                //     last_imu_time = current_imu_time;
                //     gyro_yaw_integral = 0;
                //     gyro_pitch_integral = 0;
                //     dt = 0.01f; // 初始化时设置默认时间间隔
                // }

                // if (dt > 0 && dt < 0.1f)
                // { // 有效时间间隔检查(10ms-100ms)

                //     /* 获取当前角速度 */
                //     float current_gyro_yaw = hipnuc_raw.hi91.gyr[2];   // Z轴角速度对应yaw
                //     float current_gyro_pitch = hipnuc_raw.hi91.gyr[1]; // Y轴角速度对应pitch

                //     /* 计算角加速度（用于检测突然运动） */
                //     float gyro_yaw_accel = (current_gyro_yaw - last_gyro_yaw) / dt;
                //     float gyro_pitch_accel = (current_gyro_pitch - last_gyro_pitch) / dt;

                //     /* 角度补偿分量 */
                //     float angle_yaw_compensation = -(hipnuc_raw.hi91.yaw - last_yaw);
                //     float angle_pitch_compensation = -(hipnuc_raw.hi91.pitch - last_pitch);

                //     /* 角速度积分补偿分量 */
                //     gyro_yaw_integral += current_gyro_yaw * dt;
                //     gyro_pitch_integral += current_gyro_pitch * dt;
                //     float angular_vel_yaw_compensation = -gyro_yaw_integral;
                //     float angular_vel_pitch_compensation = -gyro_pitch_integral;

                //     /* 角加速度前馈补偿（针对突然加速） */
                //     float accel_yaw_compensation = -gyro_yaw_accel * dt * dt * 0.5f;
                //     float accel_pitch_compensation = -gyro_pitch_accel * dt * dt * 0.5f;

                //     /* 动态权重调整 - 根据角加速度大小调整权重 */
                //     float yaw_accel_mag = fabsf(gyro_yaw_accel);
                //     float pitch_accel_mag = fabsf(gyro_pitch_accel);

                //     /* 当检测到突然加速时，增加角速度和加速度补偿的权重 */
                //     float yaw_angle_weight = 0.2f;
                //     float yaw_vel_weight = 0.6f;
                //     float yaw_accel_weight = 0.2f;

                //     float pitch_angle_weight = 0.2f;
                //     float pitch_vel_weight = 0.6f;
                //     float pitch_accel_weight = 0.2f;

                //     /* 突然加速检测阈值 */
                //     static const float accel_threshold = 50.0f; // 角加速度阈值(度/秒²)

                //     if (yaw_accel_mag > accel_threshold) {
                //         yaw_angle_weight = 0.1f;
                //         yaw_vel_weight = 0.7f;
                //         yaw_accel_weight = 0.2f;
                //     }

                //     if (pitch_accel_mag > accel_threshold) {
                //         pitch_angle_weight = 0.1f;
                //         pitch_vel_weight = 0.7f;
                //         pitch_accel_weight = 0.2f;
                //     }

                //     /* 三重融合补偿算法 */
                //     float fusion_yaw_compensation = yaw_angle_weight * angle_yaw_compensation +
                //                                    yaw_vel_weight * angular_vel_yaw_compensation +
                //                                    yaw_accel_weight * accel_yaw_compensation;

                //     float fusion_pitch_compensation = pitch_angle_weight * angle_pitch_compensation +
                //                                      pitch_vel_weight * angular_vel_pitch_compensation +
                //                                      pitch_accel_weight * accel_pitch_compensation;

                //     /* 限制补偿幅度，防止过补偿 */
                //     static const float max_compensation = 10.0f; // 最大补偿角度(度)
                //     fusion_yaw_compensation = fmaxf(-max_compensation, fminf(max_compensation, fusion_yaw_compensation));
                //     fusion_pitch_compensation = fmaxf(-max_compensation, fminf(max_compensation, fusion_pitch_compensation));

                //     /* 应用融合补偿 */
                //     laser_gimbal_adjust_angle(tracker->gimbal, fusion_yaw_compensation, -fusion_pitch_compensation);

                //     /* 积分泄漏，防止长期累积误差 */
                //     static const float leak_factor = 0.995f;
                //     gyro_yaw_integral *= leak_factor;
                //     gyro_pitch_integral *= leak_factor;

                //     /* 调试输出 (每500ms输出一次) */
                //     static uint32_t last_debug_time = 0;
                //     if (current_imu_time - last_debug_time > 500)
                //     {
                //         LOG_D("IMU Enhanced: Angle[%.2f,%.2f]° Vel[%.2f,%.2f]°/s Accel[%.1f,%.1f]°/s² Final[%.2f,%.2f]°",
                //               angle_yaw_compensation, angle_pitch_compensation,
                //               current_gyro_yaw, current_gyro_pitch,
                //               gyro_yaw_accel, gyro_pitch_accel,
                //               fusion_yaw_compensation, fusion_pitch_compensation);
                //         last_debug_time = current_imu_time;
                //     }

                //     /* 更新历史值 */
                //     last_gyro_yaw = current_gyro_yaw;
                //     last_gyro_pitch = current_gyro_pitch;
                // }

                // last_yaw = hipnuc_raw.hi91.yaw;
                // last_pitch = hipnuc_raw.hi91.pitch;
                // last_imu_time = current_imu_time;
                // }

                /* 添加陀螺仪补偿 */
                extern hipnuc_raw_t hipnuc_raw; // 原始数据结构体
                gyro_x = hipnuc_raw.hi91.gyr[0];
                gyro_y = hipnuc_raw.hi91.gyr[1];
                gyro_z = -hipnuc_raw.hi91.gyr[2];

                accel_x = hipnuc_raw.hi91.acc[0];
                accel_y = hipnuc_raw.hi91.acc[1];
                accel_z = hipnuc_raw.hi91.acc[2];

                yaw = -hipnuc_raw.hi91.yaw;
                static float last_yaw = 0;
                static int init_cnt = 0;
                float adj_angle = 0;
                if (init_cnt++ < 100)
                {

                    last_yaw = yaw;
                }

                uint32_t current_time = rt_tick_get_millisecond();
                static uint32_t last_comp_log_time = 0;
                static uint32_t last_time = 0;

                adj_angle = (yaw - last_yaw)*1000/(current_time-last_time);
                LOG_D("accel_x:%f", accel_x);
                // laser_gimbal_adjust_angle(tracker->gimbal,(yaw - last_yaw) , 0);


                adj_x_yaw = accel_x ;
                if(fabs(adj_x_yaw) <0.1f) adj_x_yaw = 0.f;
                last_yaw = yaw;

                last_time = current_time;
                /* 陀螺仪补偿参数 - 可以根据需要调整 */
                static float gyro_compensation_gain_yaw = 1.0f;   // yaw轴补偿增益
                static float gyro_compensation_gain_pitch = 0.3f; // pitch轴补偿增益
                static float gyro_offset_yaw = 0.0f;              // yaw轴零偏
                static float gyro_offset_pitch = 0.0f;            // pitch轴零偏

                /* 应用零偏校正 */
                float corrected_gyro_yaw = gyro_z - gyro_offset_yaw;     // Z轴对应yaw //反向安装
                float corrected_gyro_pitch = gyro_y - gyro_offset_pitch; // Y轴对应pitch

                /* 只在载体运动较大时才应用补偿 */
                if (fabs(corrected_gyro_yaw) > 0.2f || fabs(corrected_gyro_pitch) > 0.2f)
                {

                    /* 计算补偿量（反向补偿载体运动） */
                    float gyro_compensation_yaw = -corrected_gyro_yaw * gyro_compensation_gain_yaw;
                    float gyro_compensation_pitch = -corrected_gyro_pitch * gyro_compensation_gain_pitch;

                    /* 将陀螺仪补偿添加到PID输出 */
                    // fix_yaw = gyro_compensation_yaw * (current_time - last_comp_log_time);
                    fix_yaw = gyro_compensation_yaw;

                    /* 限制输出范围 */
                    fix_yaw = MAX(-60.0f, MIN(60.0f, fix_yaw));

                    if (current_time - last_comp_log_time > 1000) // 每秒打印一次补偿信息
                    {
                        LOG_D("Vision+Gyro: PID=[%.1f,0]°/s, Gyro=[%.1f,0]°/s, Final=[%.1f,%.1f]°/s",
                              fix_yaw - gyro_compensation_yaw,
                              gyro_compensation_yaw, gyro_compensation_pitch,
                              fix_yaw);
                        last_comp_log_time = current_time;
                    }
                }

                //     /* 处理正常追踪状态 */
                if (tracker->state == VISION_TRACK_STATE_TRACKING ||
                    tracker->state == VISION_TRACK_STATE_LOCKED)
                {
                    vision_tracker_process_target(tracker, &yaw_out, &pitch_out);
                }
                /* 处理自动搜索状态 */
                else if (tracker->state == VISION_TRACK_STATE_AUTO_SEARCHING)
                {
                    // vision_tracker_process_auto_search(tracker);
                    //  vision_tracker_apply_control(tracker, 5, 0);
                }

                vision_tracker_apply_control(tracker, yaw_out+fix_yaw * kp_vel_yaw + adj_angle * kp_angle_yaw +adj_x_yaw*kp_acc_x_yaw, pitch_out);

                rt_mutex_release(tracker->tracker_mutex);
            }
        }

        rt_thread_mdelay(2); // 500Hz 控制频率
    }

    LOG_I("Vision tracking thread stopped");
}

 static void acc_kp(int argc, char **argv)
{
    if (argc == 2)
    {
        kp_acc_x_yaw = atof(argv[1]);
    }
}
MSH_CMD_EXPORT(acc_kp, Test sudden acceleration compensation);
static void ang_kp(int argc, char **argv)
{
    if (argc == 2)
    {
        kp_angle_yaw = atof(argv[1]);
    }
}
MSH_CMD_EXPORT(ang_kp, Test sudden acceleration compensation);

static void vel_kp(int argc, char **argv)
{
    if (argc == 2)
    {
        kp_vel_yaw = atof(argv[1]);
    }
}
MSH_CMD_EXPORT(vel_kp, Test sudden acceleration compensation);

/* ======================== 陀螺仪补偿相关函数 ======================== */

/**
 * @brief 启用/禁用视觉追踪器的陀螺仪补偿
 */
rt_err_t vision_tracker_enable_gyro_compensation(vision_tracker_t *tracker, bool enable)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, 100) != RT_EOK)
    {
        LOG_E("Failed to take tracker mutex");
        return -RT_ETIMEOUT;
    }

    tracker->gyro_compensation_enabled = enable;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Vision tracker gyro compensation %s", enable ? "enabled" : "disabled");

    return RT_EOK;
}

/**
 * @brief 设置视觉追踪器的陀螺仪补偿参数
 */
rt_err_t vision_tracker_set_gyro_compensation_params(vision_tracker_t *tracker,
                                                     float yaw_gain,
                                                     float pitch_gain,
                                                     float motion_threshold)
{
    RT_ASSERT(tracker != RT_NULL);

    if (yaw_gain < 0.0f || yaw_gain > 2.0f ||
        pitch_gain < 0.0f || pitch_gain > 2.0f ||
        motion_threshold < 0.0f || motion_threshold > 100.0f)
    {
        LOG_E("Invalid gyro compensation parameters");
        return -RT_EINVAL;
    }

    if (rt_mutex_take(tracker->tracker_mutex, 100) != RT_EOK)
    {
        LOG_E("Failed to take tracker mutex");
        return -RT_ETIMEOUT;
    }

    tracker->gyro_compensation_gain_yaw = yaw_gain;
    tracker->gyro_compensation_gain_pitch = pitch_gain;
    tracker->gyro_motion_threshold = motion_threshold;

    rt_mutex_release(tracker->tracker_mutex);

    LOG_I("Vision tracker gyro compensation params set: yaw_gain=%.2f, pitch_gain=%.2f, threshold=%.1f°/s",
          yaw_gain, pitch_gain, motion_threshold);

    return RT_EOK;
}

/**
 * @brief 获取视觉追踪器的陀螺仪补偿参数
 */
rt_err_t vision_tracker_get_gyro_compensation_params(vision_tracker_t *tracker,
                                                     bool *enabled,
                                                     float *yaw_gain,
                                                     float *pitch_gain,
                                                     float *motion_threshold)
{
    RT_ASSERT(tracker != RT_NULL);

    if (rt_mutex_take(tracker->tracker_mutex, 100) != RT_EOK)
    {
        LOG_E("Failed to take tracker mutex");
        return -RT_ETIMEOUT;
    }

    if (enabled)
        *enabled = tracker->gyro_compensation_enabled;
    if (yaw_gain)
        *yaw_gain = tracker->gyro_compensation_gain_yaw;
    if (pitch_gain)
        *pitch_gain = tracker->gyro_compensation_gain_pitch;
    if (motion_threshold)
        *motion_threshold = tracker->gyro_motion_threshold;

    rt_mutex_release(tracker->tracker_mutex);

    return RT_EOK;
}
