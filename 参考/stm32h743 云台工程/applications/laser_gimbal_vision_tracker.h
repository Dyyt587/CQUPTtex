/**
 * @file laser_gimbal_vision_tracker.h
 * @brief 基于视觉的激光云台追踪控制系统
 * @version 1.0
 * @date 2025-07-30
 * @author Your Name
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 实现通过摄像头xy坐标，使用PID控制云台yaw和pitch角度
 * 支持目标追踪、PID参数调节、多种追踪模式
 */

#ifndef LASER_GIMBAL_VISION_TRACKER_H
#define LASER_GIMBAL_VISION_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <rtthread.h>
#include "laser_gimbal.h"
#include "apid.h"  // 使用项目中的APID库

/* 视觉追踪器配置参数 */
#define VISION_TRACKER_DEFAULT_CAMERA_WIDTH     640     // 默认摄像头宽度
#define VISION_TRACKER_DEFAULT_CAMERA_HEIGHT    480     // 默认摄像头高度
#define VISION_TRACKER_DEFAULT_FOV_H           60.0f    // 默认水平视场角(度)
#define VISION_TRACKER_DEFAULT_FOV_V           45.0f    // 默认垂直视场角(度)
#define VISION_TRACKER_MAX_ERROR_THRESHOLD     50.0f    // 最大误差阈值(像素)
#define VISION_TRACKER_TARGET_LOST_TIMEOUT     5000     // 目标丢失超时(ms)

/* PID控制器默认参数 */
#define VISION_PID_DEFAULT_KP_YAW              5.657f//0.028f     // Yaw轴比例系数
#define VISION_PID_DEFAULT_KI_YAW              0.01f    // Yaw轴积分系数
#define VISION_PID_DEFAULT_KD_YAW              0.4263f//0.0067f    // Yaw轴微分系数

#define VISION_PID_DEFAULT_KP_PITCH            5.909f     // Pitch轴比例系数
#define VISION_PID_DEFAULT_KI_PITCH            0.00f    // Pitch轴积分系数
#define VISION_PID_DEFAULT_KD_PITCH            0.342f    // Pitch轴微分系数

#define VISION_PID_MAX_OUTPUT                  50.f    // PID输出限制(度/秒)
#define VISION_PID_BISO_MAX                    0.05f    // PID误差限制(度/秒)
#define VISION_PID_DEADZONE                    2.0f     // 死区(像素)

/* 视觉追踪模式 */
typedef enum {
    VISION_TRACK_MODE_DISABLED = 0,    // 禁用追踪
    VISION_TRACK_MODE_MANUAL,          // 手动模式(单次调整)
    VISION_TRACK_MODE_CONTINUOUS,      // 连续追踪模式
    VISION_TRACK_MODE_LOCK_ON          // 锁定模式(高精度追踪)
} vision_track_mode_t;

/* 视觉追踪状态 */
typedef enum {
    VISION_TRACK_STATE_IDLE = 0,       // 空闲状态
    VISION_TRACK_STATE_SEARCHING,      // 搜索目标
    VISION_TRACK_STATE_AUTO_SEARCHING, // 自动搜索模式(低可信度触发)
    VISION_TRACK_STATE_TRACKING,       // 正在追踪
    VISION_TRACK_STATE_LOCKED,         // 已锁定目标
    VISION_TRACK_STATE_TARGET_LOST,    // 目标丢失
    VISION_TRACK_STATE_ERROR           // 错误状态
} vision_track_state_t;

/* 摄像头像素坐标结构体 */
typedef struct {
    float x;        // 像素X坐标 (0 到 camera_width)
    float y;        // 像素Y坐标 (0 到 camera_height)
    bool valid;     // 坐标是否有效
    uint32_t timestamp;  // 时间戳(ms)
} vision_pixel_coord_t;

/* 目标信息结构体 */
typedef struct {
    vision_pixel_coord_t pixel_coord;   // 像素坐标
    float confidence;                   // 置信度 (0.0-1.0)
    float size;                        // 目标大小 (像素面积或直径)
    uint32_t track_id;                 // 追踪ID
} vision_target_t;

/* PID控制器结构体 - 使用APID库 */
typedef struct {
    apid_t controller;          // APID控制器实例
    float error;                // 当前误差
    float output;               // PID输出
    uint32_t last_time;         // 上次计算时间
} vision_pid_controller_t;

/* 摄像头参数结构体 */
typedef struct {
    uint16_t width;         // 摄像头宽度(像素)
    uint16_t height;        // 摄像头高度(像素)
    float fov_horizontal;   // 水平视场角(度)
    float fov_vertical;     // 垂直视场角(度)
    float center_x;         // 图像中心X坐标
    float center_y;         // 图像中心Y坐标
} vision_camera_params_t;

/* 视觉追踪器结构体 */
typedef struct {
    /* 基本参数 */
    volatile vision_track_mode_t mode;           // 追踪模式
    volatile vision_track_state_t state;         // 追踪状态
    bool enabled;                       // 是否启用
    
    /* 云台控制 */
    laser_gimbal_t *gimbal;             // 云台设备指针
    
    /* 摄像头参数 */
    vision_camera_params_t camera;      // 摄像头参数
    
    /* PID控制器 */
    vision_pid_controller_t pid_yaw;    // Yaw轴PID控制器
    vision_pid_controller_t pid_pitch;  // Pitch轴PID控制器
    
    /* 目标信息 */
    vision_target_t current_target;     // 当前目标
    vision_target_t target_history[5];  // 目标历史(用于滤波)
    uint8_t history_index;              // 历史索引
    uint8_t history_count;              // 历史数量
    
    /* 追踪参数 */
    float deadzone_pixels;              // 死区(像素)
    float max_error_threshold;          // 最大误差阈值
    uint32_t target_lost_timeout;       // 目标丢失超时
    uint32_t last_target_time;          // 上次目标时间
    
    /* 目标偏移参数 */
    float offset_x;                     // X轴偏移量(像素)
    float offset_y;                     // Y轴偏移量(像素)
    bool offset_enabled;                // 是否启用偏移
    
    /* 自动搜索参数 */
    float confidence_threshold;         // 可信度阈值(低于此值触发自动搜索)
    bool auto_search_enabled;           // 是否启用自动搜索
    float search_yaw_speed;             // 搜索时yaw轴旋转速度(度/秒)
    float search_pitch_angle;           // 搜索时pitch轴锁定角度(度)
    uint32_t search_start_time;         // 搜索开始时间
    float search_start_yaw;             // 搜索开始时的yaw角度
    uint32_t max_search_time;           // 最大搜索时间(ms)
    bool search_completed_full_cycle;   // 是否完成一圈搜索
    uint32_t search_cooldown_time;      // 搜索冷却时间(ms)
    uint32_t last_search_end_time;      // 上次搜索结束时间
    
    /* 陀螺仪补偿参数 */
    bool gyro_compensation_enabled;     // 是否启用陀螺仪补偿
    float gyro_compensation_gain_yaw;   // Yaw轴陀螺仪补偿增益
    float gyro_compensation_gain_pitch; // Pitch轴陀螺仪补偿增益
    float gyro_motion_threshold;        // 运动阈值(度/秒)
    uint32_t gyro_last_log_time;        // 上次陀螺仪日志时间
    
    /* 统计信息 */
    uint32_t track_count;               // 追踪次数
    uint32_t success_count;             // 成功次数
    uint32_t lost_count;                // 丢失次数
    
    /* 同步控制 */
    rt_mutex_t tracker_mutex;           // 追踪器互斥锁
    rt_thread_t track_thread;           // 追踪线程
    bool thread_running;                // 线程运行标志
    
    /* 回调函数 */
    void (*on_target_acquired)(vision_target_t target);    // 目标获取回调
    void (*on_target_lost)(void);                          // 目标丢失回调
    void (*on_tracking_error)(int error_code);             // 追踪错误回调
    
} vision_tracker_t;

/* ======================== 核心控制函数 ======================== */

/**
 * @brief 初始化视觉追踪器
 * @param tracker 追踪器结构体指针
 * @param gimbal 云台设备指针
 * @param camera_width 摄像头宽度
 * @param camera_height 摄像头高度
 * @param fov_h 水平视场角(度)
 * @param fov_v 垂直视场角(度)
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_init(vision_tracker_t *tracker,
                            laser_gimbal_t *gimbal,
                            uint16_t camera_width,
                            uint16_t camera_height,
                            float fov_h,
                            float fov_v);

/**
 * @brief 反初始化视觉追踪器
 * @param tracker 追踪器结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_deinit(vision_tracker_t *tracker);

/**
 * @brief 启用/禁用视觉追踪
 * @param tracker 追踪器结构体指针
 * @param enable 是否启用
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_enable(vision_tracker_t *tracker, bool enable);

/**
 * @brief 设置追踪模式
 * @param tracker 追踪器结构体指针
 * @param mode 追踪模式
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_mode(vision_tracker_t *tracker, vision_track_mode_t mode);

/* ======================== 目标输入函数 ======================== */

/**
 * @brief 输入目标像素坐标
 * @param tracker 追踪器结构体指针
 * @param pixel_x 像素X坐标
 * @param pixel_y 像素Y坐标
 * @param confidence 置信度 (0.0-1.0)
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_input_target(vision_tracker_t *tracker,
                                    float pixel_x,
                                    float pixel_y,
                                    float confidence);

/**
 * @brief 输入完整目标信息
 * @param tracker 追踪器结构体指针
 * @param target 目标信息
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_input_target_info(vision_tracker_t *tracker,
                                         vision_target_t *target);

/**
 * @brief 标记目标丢失
 * @param tracker 追踪器结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_target_lost(vision_tracker_t *tracker);

/* ======================== PID参数配置函数 ======================== */

/**
 * @brief 设置Yaw轴PID参数
 * @param tracker 追踪器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_yaw_pid(vision_tracker_t *tracker,
                                   float kp, float ki, float kd);

/**
 * @brief 设置Pitch轴PID参数
 * @param tracker 追踪器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_pitch_pid(vision_tracker_t *tracker,
                                     float kp, float ki, float kd);

/**
 * @brief 设置PID输出限制和积分限制
 * @param tracker 追踪器结构体指针
 * @param max_output 最大输出(度/秒)
 * @param max_integral 最大积分值
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_pid_limits(vision_tracker_t *tracker,
                                      float max_output,
                                      float max_integral);

/* ======================== 追踪参数配置函数 ======================== */

/**
 * @brief 设置追踪参数
 * @param tracker 追踪器结构体指针
 * @param deadzone 死区(像素)
 * @param max_error 最大误差阈值(像素)
 * @param lost_timeout 目标丢失超时(ms)
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_track_params(vision_tracker_t *tracker,
                                        float deadzone,
                                        float max_error,
                                        uint32_t lost_timeout);

/**
 * @brief 设置目标偏移量
 * @param tracker 追踪器结构体指针
 * @param offset_x X轴偏移量(像素，正值向右偏移)
 * @param offset_y Y轴偏移量(像素，正值向下偏移)
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_offset(vision_tracker_t *tracker,
                                  float offset_x,
                                  float offset_y);

/**
 * @brief 启用/禁用目标偏移
 * @param tracker 追踪器结构体指针
 * @param enable 是否启用偏移
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_enable_offset(vision_tracker_t *tracker, bool enable);

/**
 * @brief 获取当前偏移设置
 * @param tracker 追踪器结构体指针
 * @param offset_x 返回X轴偏移量
 * @param offset_y 返回Y轴偏移量
 * @param enabled 返回偏移是否启用
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_get_offset(vision_tracker_t *tracker,
                                  float *offset_x,
                                  float *offset_y,
                                  bool *enabled);

/* ======================== 自动搜索函数 ======================== */

/**
 * @brief 设置自动搜索参数
 * @param tracker 追踪器结构体指针
 * @param confidence_threshold 可信度阈值(低于此值触发搜索)
 * @param search_speed yaw轴搜索速度(度/秒)
 * @param pitch_angle 搜索时pitch轴锁定角度(度)
 * @param max_search_time 最大搜索时间(ms)
 * @param cooldown_time 搜索冷却时间(ms)
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_auto_search_params(vision_tracker_t *tracker,
                                              float confidence_threshold,
                                              float search_speed,
                                              float pitch_angle,
                                              uint32_t max_search_time,
                                              uint32_t cooldown_time);

/**
 * @brief 启用/禁用自动搜索功能
 * @param tracker 追踪器结构体指针
 * @param enable 是否启用自动搜索
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_enable_auto_search(vision_tracker_t *tracker, bool enable);

/**
 * @brief 获取自动搜索设置
 * @param tracker 追踪器结构体指针
 * @param confidence_threshold 返回可信度阈值
 * @param search_speed 返回搜索速度
 * @param pitch_angle 返回pitch锁定角度
 * @param enabled 返回是否启用
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_get_auto_search_params(vision_tracker_t *tracker,
                                               float *confidence_threshold,
                                               float *search_speed,
                                               float *pitch_angle,
                                               uint32_t *cooldown_time,
                                               bool *enabled);

/**
 * @brief 手动触发自动搜索
 * @param tracker 追踪器结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_start_auto_search(vision_tracker_t *tracker);

/**
 * @brief 停止自动搜索
 * @param tracker 追踪器结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_stop_auto_search(vision_tracker_t *tracker);

/* ======================== 状态查询函数 ======================== */

/**
 * @brief 获取追踪状态
 * @param tracker 追踪器结构体指针
 * @return vision_track_state_t 追踪状态
 */
vision_track_state_t vision_tracker_get_state(vision_tracker_t *tracker);

/**
 * @brief 获取当前目标信息
 * @param tracker 追踪器结构体指针
 * @param target 返回的目标信息
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_get_current_target(vision_tracker_t *tracker,
                                          vision_target_t *target);

/**
 * @brief 获取PID误差信息
 * @param tracker 追踪器结构体指针
 * @param yaw_error 返回Yaw轴误差
 * @param pitch_error 返回Pitch轴误差
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_get_pid_errors(vision_tracker_t *tracker,
                                      float *yaw_error,
                                      float *pitch_error);

/**
 * @brief 获取统计信息
 * @param tracker 追踪器结构体指针
 * @param track_count 追踪次数
 * @param success_count 成功次数
 * @param lost_count 丢失次数
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_get_statistics(vision_tracker_t *tracker,
                                      uint32_t *track_count,
                                      uint32_t *success_count,
                                      uint32_t *lost_count);

/* ======================== 实用工具函数 ======================== */

/**
 * @brief 像素坐标转角度误差
 * @param tracker 追踪器结构体指针
 * @param pixel_x 像素X坐标
 * @param pixel_y 像素Y坐标
 * @param yaw_error 返回Yaw轴角度误差(度)
 * @param pitch_error 返回Pitch轴角度误差(度)
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_pixel_to_angle_error(vision_tracker_t *tracker,
                                            float pixel_x,
                                            float pixel_y,
                                            float *yaw_error,
                                            float *pitch_error);

/**
 * @brief 重置PID控制器
 * @param tracker 追踪器结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_reset_pid(vision_tracker_t *tracker);

/**
 * @brief 显示追踪器状态信息
 * @param tracker 追踪器结构体指针
 */
void vision_tracker_show_status(vision_tracker_t *tracker);

/* ======================== 陀螺仪补偿函数 ======================== */

/**
 * @brief 启用/禁用视觉追踪器的陀螺仪补偿
 * @param tracker 追踪器结构体指针
 * @param enable 是否启用
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_enable_gyro_compensation(vision_tracker_t *tracker, bool enable);

/**
 * @brief 设置视觉追踪器的陀螺仪补偿参数
 * @param tracker 追踪器结构体指针
 * @param yaw_gain Yaw轴补偿增益
 * @param pitch_gain Pitch轴补偿增益
 * @param motion_threshold 运动阈值(度/秒)
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_set_gyro_compensation_params(vision_tracker_t *tracker,
                                                    float yaw_gain,
                                                    float pitch_gain,
                                                    float motion_threshold);

/**
 * @brief 获取视觉追踪器的陀螺仪补偿参数
 * @param tracker 追踪器结构体指针
 * @param enabled 返回是否启用
 * @param yaw_gain 返回Yaw轴补偿增益
 * @param pitch_gain 返回Pitch轴补偿增益
 * @param motion_threshold 返回运动阈值
 * @return rt_err_t 错误码
 */
rt_err_t vision_tracker_get_gyro_compensation_params(vision_tracker_t *tracker,
                                                    bool *enabled,
                                                    float *yaw_gain,
                                                    float *pitch_gain,
                                                    float *motion_threshold);

#ifdef __cplusplus
}
#endif

#endif // LASER_GIMBAL_VISION_TRACKER_H
