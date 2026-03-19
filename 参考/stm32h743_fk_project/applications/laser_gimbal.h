/**
 * @file laser_gimbal.h
 * @brief 基于MS4010双电机的二维激光云台控制系统
 * @version 1.0
 * @date 2025-07-28
 * @author Your Name
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 实现通过输入距离和XY坐标控制激光指向指定位置的功能
 * 支持自动角度计算、平滑运动控制、多种控制模式
 */

#ifndef LASER_GIMBAL_H
#define LASER_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <rtthread.h>
#include "drv_ms4010.h"

/* 激光云台配置参数 */
#define LASER_GIMBAL_DEFAULT_HEIGHT     1.0f    // 默认云台高度(米)
#define LASER_GIMBAL_MAX_DISTANCE       50.0f   // 最大控制距离(米)
#define LASER_GIMBAL_MIN_DISTANCE       0.5f    // 最小控制距离(米)
#define LASER_GIMBAL_MAX_PITCH_ANGLE    180.0f   // 最大俯仰角(度)
#define LASER_GIMBAL_MIN_PITCH_ANGLE    -180.0f  // 最小俯仰角(度)
#define LASER_GIMBAL_MAX_YAW_ANGLE      720.0f  // 最大偏航角(度)
#define LASER_GIMBAL_MIN_YAW_ANGLE      -720.0f // 最小偏航角(度)

/* 角度单位转换 */
#define LASER_GIMBAL_DEG_TO_RAD(deg)    ((deg) * M_PI / 180.0f)
#define LASER_GIMBAL_RAD_TO_DEG(rad)    ((rad) * 180.0f / M_PI)

/* MS4010电机角度转换 (根据你的机械结构调整) */
#define LASER_GIMBAL_MOTOR_ANGLE_SCALE  100.0f  // MS4010角度单位转换系数 (0.01度)

#define M_PI 3.1415926f
/* 激光云台控制模式 */
typedef enum {
    LASER_GIMBAL_MODE_DIRECT = 0,    // 直接角度控制模式
    LASER_GIMBAL_MODE_COORDINATE,    // 坐标控制模式  
    LASER_GIMBAL_MODE_SMOOTH,        // 平滑运动模式
    LASER_GIMBAL_MODE_TRACK          // 跟踪模式
} laser_gimbal_mode_t;

/* 激光云台状态 */
typedef enum {
    LASER_GIMBAL_STATE_IDLE = 0,     // 空闲状态
    LASER_GIMBAL_STATE_MOVING,       // 运动中
    LASER_GIMBAL_STATE_TRACKING,     // 跟踪中
    LASER_GIMBAL_STATE_ERROR         // 错误状态
} laser_gimbal_state_t;

/* 三维坐标结构体 */
typedef struct {
    float x;    // X坐标 (米)
    float y;    // Y坐标 (米) 
    float z;    // Z坐标/高度 (米)
} laser_coordinate_t;

/* 云台角度结构体 */
typedef struct {
    float yaw;      // 偏航角/方位角 (度) - 水平旋转
    float pitch;    // 俯仰角 (度) - 垂直旋转
} laser_gimbal_angle_t;

/* 云台运动参数 */
typedef struct {
    float max_yaw_speed;     // 最大偏航速度 (度/秒)
    float max_pitch_speed;   // 最大俯仰速度 (度/秒)
    float acceleration;      // 加速度 (度/秒²)
    float smooth_factor;     // 平滑因子 (0.0-1.0)
} laser_gimbal_motion_params_t;

/* 激光云台设备结构体 */
typedef struct {
    /* 基本参数 */
    laser_gimbal_mode_t mode;           // 当前控制模式
    laser_gimbal_state_t state;         // 当前状态
    
    /* 电机设备 */
    ms4010_device_t *yaw_motor;         // X轴电机 (偏航轴, 水平旋转, ID=1)
    ms4010_device_t *pitch_motor;       // Y轴电机 (俯仰轴, 垂直旋转, ID=2)
    
    /* 机械结构参数 */
    float gimbal_height;                // 云台安装高度 (米)
    float yaw_offset;                   // X轴(偏航)零点偏移 (度)
    float pitch_offset;                 // Y轴(俯仰)零点偏移 (度)
    bool yaw_reverse;                   // X轴反向
    bool pitch_reverse;                 // Y轴反向
    
    /* 校准相关参数 */
    bool calibration_mode;              // 校准模式标志
    float yaw_motor_current_angle;      // X轴电机当前原始角度 (度)
    float pitch_motor_current_angle;    // Y轴电机当前原始角度 (度)
    
    /* 实时角度监控 */
    rt_thread_t calibration_monitor_thread; // 校准监控线程
    bool calibration_monitor_running;   // 监控线程运行标志
    float angle_stable_threshold;       // 角度稳定阈值 (度)
    uint32_t angle_stable_time_ms;      // 角度稳定时间要求 (毫秒)
    uint32_t last_angle_change_time;    // 上次角度改变时间
    float last_stable_yaw;              // 上次稳定的X轴角度
    float last_stable_pitch;            // 上次稳定的Y轴角度
    bool auto_save_enabled;             // 自动保存使能
    
    /* 当前状态 */
    laser_coordinate_t current_target;  // 当前目标坐标
    laser_gimbal_angle_t current_angle; // 当前角度
    laser_gimbal_angle_t target_angle;  // 目标角度
    
    /* 运动控制参数 */
    laser_gimbal_motion_params_t motion_params;
    
    /* 速度控制状态 */
    bool velocity_control_enabled;      // 速度控制模式标志
    float current_yaw_speed;            // 当前yaw轴速度 (度/秒)
    float current_pitch_speed;          // 当前pitch轴速度 (度/秒)
    uint32_t velocity_update_time;      // 速度更新时间戳
    
    /* 陀螺仪补偿参数 */
    bool gyro_compensation_enabled;     // 陀螺仪补偿使能标志
    float gyro_compensation_gain_yaw;   // yaw轴陀螺仪补偿增益
    float gyro_compensation_gain_pitch; // pitch轴陀螺仪补偿增益
    float gyro_offset_yaw;              // yaw轴陀螺仪零偏
    float gyro_offset_pitch;            // pitch轴陀螺仪零偏
    float gyro_offset_roll;             // roll轴陀螺仪零偏
    float last_gyro_yaw;                // 上次yaw轴陀螺仪数据
    float last_gyro_pitch;              // 上次pitch轴陀螺仪数据
    float last_gyro_roll;               // 上次roll轴陀螺仪数据
    uint32_t last_gyro_update_time;     // 上次陀螺仪数据更新时间
    bool gyro_calibration_mode;         // 陀螺仪校准模式
    float gyro_calibration_samples[3][100]; // 校准采样数据 [yaw/pitch/roll][samples]
    int gyro_calibration_count;         // 校准采样计数
    
    /* 限位参数 */
    laser_gimbal_angle_t angle_limits_min;  // 角度下限
    laser_gimbal_angle_t angle_limits_max;  // 角度上限
    
    /* 统计信息 */
    uint32_t command_count;             // 命令计数
    uint32_t error_count;               // 错误计数
    uint32_t last_update_time;          // 上次更新时间
    
    /* 线程同步 */
    rt_mutex_t gimbal_mutex;            // 云台互斥锁
    
    /* 回调函数 */
    void (*on_target_reached)(laser_coordinate_t target);  // 到达目标回调
    void (*on_error)(int error_code);                     // 错误回调
    
} laser_gimbal_t;

/* 函数声明 */

/**
 * @brief 初始化激光云台系统
 * @param gimbal 云台设备结构体指针
 * @param yaw_motor X轴电机设备指针 (ID=1, 偏航轴, 水平旋转)
 * @param pitch_motor Y轴电机设备指针 (ID=2, 俯仰轴, 垂直旋转)
 * @param gimbal_height 云台安装高度 (米)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_init(laser_gimbal_t *gimbal, 
                          ms4010_device_t *yaw_motor,
                          ms4010_device_t *pitch_motor,
                          float gimbal_height);

/**
 * @brief 反初始化激光云台系统
 * @param gimbal 云台设备结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_deinit(laser_gimbal_t *gimbal);

/**
 * @brief 设置云台机械参数
 * @param gimbal 云台设备结构体指针
 * @param yaw_offset 偏航轴零点偏移 (度)
 * @param pitch_offset 俯仰轴零点偏移 (度)
 * @param yaw_reverse 偏航轴是否反向
 * @param pitch_reverse 俯仰轴是否反向
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_mechanical_params(laser_gimbal_t *gimbal,
                                           float yaw_offset,
                                           float pitch_offset, 
                                           bool yaw_reverse,
                                           bool pitch_reverse);

/**
 * @brief 设置云台运动参数
 * @param gimbal 云台设备结构体指针
 * @param max_yaw_speed 最大偏航速度 (度/秒)
 * @param max_pitch_speed 最大俯仰速度 (度/秒)
 * @param acceleration 加速度 (度/秒²)
 * @param smooth_factor 平滑因子 (0.0-1.0)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_motion_params(laser_gimbal_t *gimbal,
                                       float max_yaw_speed,
                                       float max_pitch_speed,
                                       float acceleration,
                                       float smooth_factor);

/**
 * @brief 设置云台角度限制
 * @param gimbal 云台设备结构体指针
 * @param yaw_min 偏航角最小值 (度)
 * @param yaw_max 偏航角最大值 (度)
 * @param pitch_min 俯仰角最小值 (度)
 * @param pitch_max 俯仰角最大值 (度)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_angle_limits(laser_gimbal_t *gimbal,
                                      float yaw_min, float yaw_max,
                                      float pitch_min, float pitch_max);

/**
 * @brief 坐标控制 - 输入距离和XY坐标控制激光指向
 * @param gimbal 云台设备结构体指针
 * @param distance 目标距离 (米)
 * @param x 目标X坐标相对于云台 (米)
 * @param y 目标Y坐标相对于云台 (米)
 * @return rt_err_t 错误码
 * 
 * @note 坐标系定义:
 *       - X轴: 云台正前方为正方向
 *       - Y轴: 云台右侧为正方向  
 *       - Z轴: 向上为正方向
 *       - 偏航角: 绕Z轴旋转，向右为正
 *       - 俯仰角: 绕Y轴旋转，向下为正
 */
rt_err_t laser_gimbal_point_to_coordinate(laser_gimbal_t *gimbal,
                                         float distance,
                                         float x, float y);

/**
 * @brief 三维坐标控制 - 输入三维坐标控制激光指向
 * @param gimbal 云台设备结构体指针
 * @param target 目标三维坐标 (相对于云台)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_point_to_3d_coordinate(laser_gimbal_t *gimbal,
                                            laser_coordinate_t target);

/**
 * @brief 直接角度控制
 * @param gimbal 云台设备结构体指针
 * @param yaw 目标偏航角 (度)
 * @param pitch 目标俯仰角 (度)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_angle(laser_gimbal_t *gimbal,
                               float yaw, float pitch);

/**
 * @brief 获取当前云台角度
 * @param gimbal 云台设备结构体指针
 * @param angle 返回的角度结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_current_angle(laser_gimbal_t *gimbal,
                                       laser_gimbal_angle_t *angle);

/**
 * @brief 设置单独的yaw角度 (保持当前pitch不变)
 * @param gimbal 云台设备结构体指针
 * @param yaw 目标偏航角 (度)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_yaw(laser_gimbal_t *gimbal, float yaw);

/**
 * @brief 设置单独的pitch角度 (保持当前yaw不变)
 * @param gimbal 云台设备结构体指针
 * @param pitch 目标俯仰角 (度)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_pitch(laser_gimbal_t *gimbal, float pitch);

/**
 * @brief 相对角度调整 (在当前角度基础上增减)
 * @param gimbal 云台设备结构体指针
 * @param yaw_delta 偏航角增量 (度, 正值右转, 负值左转)
 * @param pitch_delta 俯仰角增量 (度, 正值向上, 负值向下)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_adjust_angle(laser_gimbal_t *gimbal, 
                                  float yaw_delta, float pitch_delta);

/**
 * @brief 平滑移动到指定角度 (可控制速度)
 * @param gimbal 云台设备结构体指针
 * @param yaw 目标偏航角 (度)
 * @param pitch 目标俯仰角 (度)
 * @param speed_factor 速度因子 (0.1-2.0, 1.0为默认速度)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_smooth_move_to_angle(laser_gimbal_t *gimbal,
                                          float yaw, float pitch, 
                                          float speed_factor);

/**
 * @brief 获取当前yaw角度
 * @param gimbal 云台设备结构体指针
 * @param yaw 返回的偏航角
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_yaw(laser_gimbal_t *gimbal, float *yaw);

/**
 * @brief 获取当前pitch角度
 * @param gimbal 云台设备结构体指针
 * @param pitch 返回的俯仰角
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_pitch(laser_gimbal_t *gimbal, float *pitch);

/* ======================== 速度控制函数 ======================== */

/**
 * @brief 设置yaw轴旋转速度 (连续旋转)
 * @param gimbal 云台设备结构体指针
 * @param speed 旋转速度 (度/秒, 正值右转, 负值左转, 0停止)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_yaw_speed(laser_gimbal_t *gimbal, float speed);

/**
 * @brief 设置pitch轴旋转速度 (连续旋转)
 * @param gimbal 云台设备结构体指针
 * @param speed 旋转速度 (度/秒, 正值向上, 负值向下, 0停止)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_pitch_speed(laser_gimbal_t *gimbal, float speed);

/**
 * @brief 同时设置yaw和pitch轴旋转速度
 * @param gimbal 云台设备结构体指针
 * @param yaw_speed yaw轴旋转速度 (度/秒)
 * @param pitch_speed pitch轴旋转速度 (度/秒)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_angular_velocity(laser_gimbal_t *gimbal, 
                                          float yaw_speed, float pitch_speed);

/**
 * @brief 获取当前yaw轴旋转速度
 * @param gimbal 云台设备结构体指针
 * @param speed 返回的旋转速度 (度/秒)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_yaw_speed(laser_gimbal_t *gimbal, float *speed);

/**
 * @brief 获取当前pitch轴旋转速度
 * @param gimbal 云台设备结构体指针
 * @param speed 返回的旋转速度 (度/秒)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_pitch_speed(laser_gimbal_t *gimbal, float *speed);

/**
 * @brief 停止所有轴的速度控制 (回到位置控制模式)
 * @param gimbal 云台设备结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_stop_velocity_control(laser_gimbal_t *gimbal);

/* ======================== 陀螺仪补偿函数 ======================== */

/**
 * @brief 启用/禁用陀螺仪补偿功能
 * @param gimbal 云台设备结构体指针
 * @param enable 是否启用陀螺仪补偿
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_enable_gyro_compensation(laser_gimbal_t *gimbal, bool enable);

/**
 * @brief 设置陀螺仪补偿增益
 * @param gimbal 云台设备结构体指针
 * @param yaw_gain yaw轴补偿增益
 * @param pitch_gain pitch轴补偿增益
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_gyro_compensation_gain(laser_gimbal_t *gimbal, 
                                                float yaw_gain, float pitch_gain);

/**
 * @brief 开始陀螺仪零偏校准
 * @param gimbal 云台设备结构体指针
 * @param samples 校准采样数量 (最大100)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_start_gyro_calibration(laser_gimbal_t *gimbal, int samples);

/**
 * @brief 停止陀螺仪零偏校准并计算结果
 * @param gimbal 云台设备结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_finish_gyro_calibration(laser_gimbal_t *gimbal);

/**
 * @brief 手动设置陀螺仪零偏
 * @param gimbal 云台设备结构体指针
 * @param yaw_offset yaw轴零偏 (度/秒)
 * @param pitch_offset pitch轴零偏 (度/秒)
 * @param roll_offset roll轴零偏 (度/秒)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_gyro_offset(laser_gimbal_t *gimbal, 
                                     float yaw_offset, float pitch_offset, float roll_offset);

/**
 * @brief 获取陀螺仪补偿参数
 * @param gimbal 云台设备结构体指针
 * @param enabled 返回补偿是否启用
 * @param yaw_gain 返回yaw轴增益
 * @param pitch_gain 返回pitch轴增益
 * @param yaw_offset 返回yaw轴零偏
 * @param pitch_offset 返回pitch轴零偏
 * @param roll_offset 返回roll轴零偏
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_gyro_compensation_params(laser_gimbal_t *gimbal,
                                                  bool *enabled,
                                                  float *yaw_gain, float *pitch_gain,
                                                  float *yaw_offset, float *pitch_offset, float *roll_offset);

/**
 * @brief 更新陀螺仪数据并应用补偿
 * @param gimbal 云台设备结构体指针
 * @param gyro_yaw yaw轴陀螺仪数据 (度/秒)
 * @param gyro_pitch pitch轴陀螺仪数据 (度/秒)
 * @param gyro_roll roll轴陀螺仪数据 (度/秒)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_update_gyro_compensation(laser_gimbal_t *gimbal,
                                              float gyro_yaw, float gyro_pitch, float gyro_roll);

/* ======================== 基本控制函数 ======================== */

/**
 * @brief 回零操作
 * @param gimbal 云台设备结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_home(laser_gimbal_t *gimbal);

/**
 * @brief 停止云台运动
 * @param gimbal 云台设备结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_stop(laser_gimbal_t *gimbal);

/**
 * @brief 启用/禁用电机
 * @param gimbal 云台设备结构体指针
 * @param enable true:启用, false:禁用
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_enable(laser_gimbal_t *gimbal, bool enable);

/* 角度校准功能 */

/**
 * @brief 进入校准模式 - 禁用角度转换，直接控制电机
 * @param gimbal 云台设备结构体指针
 * @param enable true:进入校准模式, false:退出校准模式
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_calibration_mode(laser_gimbal_t *gimbal, bool enable);

/**
 * @brief 校准模式下直接设置电机角度 (不应用偏移)
 * @param gimbal 云台设备结构体指针
 * @param yaw_motor_angle X轴电机原始角度 (度)
 * @param pitch_motor_angle Y轴电机原始角度 (度)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_calibration_set_motor_angle(laser_gimbal_t *gimbal,
                                                  float yaw_motor_angle,
                                                  float pitch_motor_angle);

/**
 * @brief 校准模式下微调电机角度
 * @param gimbal 云台设备结构体指针
 * @param yaw_delta X轴微调量 (度)
 * @param pitch_delta Y轴微调量 (度)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_calibration_adjust_angle(laser_gimbal_t *gimbal,
                                               float yaw_delta,
                                               float pitch_delta);

/**
 * @brief 获取当前电机原始角度 (用于校准)
 * @param gimbal 云台设备结构体指针
 * @param yaw_motor_angle 返回X轴电机原始角度
 * @param pitch_motor_angle 返回Y轴电机原始角度
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_motor_raw_angle(laser_gimbal_t *gimbal,
                                         float *yaw_motor_angle,
                                         float *pitch_motor_angle);

/**
 * @brief 计算并设置角度偏移 (当云台指向正前方0°位置时调用)
 * @param gimbal 云台设备结构体指针
 * @param target_yaw 期望的X轴角度 (通常为0°)
 * @param target_pitch 期望的Y轴角度 (通常为0°)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_calibrate_offset(laser_gimbal_t *gimbal,
                                      float target_yaw,
                                      float target_pitch);

/**
 * @brief 保存校准参数到文件
 * @param gimbal 云台设备结构体指针
 * @param filename 文件名 (如果为NULL则使用默认文件名)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_save_calibration(laser_gimbal_t *gimbal, const char *filename);

/**
 * @brief 从文件加载校准参数
 * @param gimbal 云台设备结构体指针
 * @param filename 文件名 (如果为NULL则使用默认文件名)
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_load_calibration(laser_gimbal_t *gimbal, const char *filename);

/**
 * @brief 查看校准参数存储状态
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_show_calibration_storage(void);

/**
 * @brief 重置校准参数存储
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_reset_calibration_storage(void);

/**
 * @brief 设置校准监控参数
 * @param gimbal 云台设备结构体指针
 * @param stable_threshold 角度稳定阈值 (度) - 角度变化小于此值认为稳定
 * @param stable_time_ms 稳定时间要求 (毫秒) - 稳定多久后触发保存
 * @param auto_save 是否启用自动保存
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_set_calibration_monitor_params(laser_gimbal_t *gimbal,
                                                     float stable_threshold,
                                                     uint32_t stable_time_ms,
                                                     bool auto_save);

/**
 * @brief 启动校准角度监控 (内部函数，校准模式时自动调用)
 * @param gimbal 云台设备结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_start_calibration_monitor(laser_gimbal_t *gimbal);

/**
 * @brief 停止校准角度监控 (内部函数，退出校准模式时自动调用)
 * @param gimbal 云台设备结构体指针
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_stop_calibration_monitor(laser_gimbal_t *gimbal);

/**
 * @brief 获取云台状态信息
 * @param gimbal 云台设备结构体指针
 * @return laser_gimbal_state_t 当前状态
 */
laser_gimbal_state_t laser_gimbal_get_state(laser_gimbal_t *gimbal);

/**
 * @brief 获取云台统计信息
 * @param gimbal 云台设备结构体指针
 * @param command_count 返回命令计数
 * @param error_count 返回错误计数
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_get_statistics(laser_gimbal_t *gimbal,
                                    uint32_t *command_count,
                                    uint32_t *error_count);

/* 辅助函数 */

/**
 * @brief 坐标转角度计算
 * @param target 目标坐标
 * @param gimbal_height 云台高度
 * @param angle 返回的角度
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_coordinate_to_angle(laser_coordinate_t target,
                                         float gimbal_height,
                                         laser_gimbal_angle_t *angle);

/**
 * @brief 角度转坐标计算 (用于验证)
 * @param angle 角度
 * @param distance 距离
 * @param gimbal_height 云台高度
 * @param coordinate 返回的坐标
 * @return rt_err_t 错误码
 */
rt_err_t laser_gimbal_angle_to_coordinate(laser_gimbal_angle_t angle,
                                         float distance,
                                         float gimbal_height,
                                         laser_coordinate_t *coordinate);

/**
 * @brief 角度限制检查
 * @param gimbal 云台设备结构体指针
 * @param angle 要检查的角度
 * @return bool true:在限制范围内, false:超出限制
 */
bool laser_gimbal_check_angle_limits(laser_gimbal_t *gimbal,
                                    laser_gimbal_angle_t angle);

#ifdef __cplusplus
}
#endif

#endif // LASER_GIMBAL_H
