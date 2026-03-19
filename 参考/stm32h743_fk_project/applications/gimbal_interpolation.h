#ifndef GIMBAL_INTERPOLATION_H
#define GIMBAL_INTERPOLATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <rtthread.h>
#include "drv_emm_v5.h"

// 二维云台插补算法头文件

// 插补类型枚举
typedef enum {
    INTERP_TYPE_LINE = 0,    // 直线插补
    INTERP_TYPE_ARC_CW,      // 顺时针圆弧插补
    INTERP_TYPE_ARC_CCW      // 逆时针圆弧插补
} interpolation_type_t;

// 插补状态枚举
typedef enum {
    INTERP_STATE_IDLE = 0,   // 空闲状态
    INTERP_STATE_RUNNING,    // 运行中
    INTERP_STATE_FINISHED,   // 已完成
    INTERP_STATE_ERROR       // 错误状态
} interpolation_state_t;

// 坐标点结构体
typedef struct {
    float x;    // X轴坐标 (单位：度或脉冲)
    float y;    // Y轴坐标 (单位：度或脉冲)
} point_t;

// 直线插补参数结构体
typedef struct {
    point_t start_pos;     // 起始点
    point_t end_pos;       // 终点
    float feed_rate;       // 进给速度 (度/秒)
    uint32_t total_steps;  // 总步数
    uint32_t current_step; // 当前步数
} line_interp_t;

// 圆弧插补参数结构体
typedef struct {
    point_t start_pos;     // 起始点
    point_t end_pos;       // 终点
    point_t center_pos;    // 圆心坐标
    float radius;          // 半径
    float start_angle;     // 起始角度 (弧度)
    float end_angle;       // 终止角度 (弧度)
    float angle_span;      // 角度跨度
    float feed_rate;       // 进给速度 (度/秒)
    uint32_t total_steps;  // 总步数
    uint32_t current_step; // 当前步数
    bool clockwise;        // 是否顺时针
} arc_interp_t;

// 插补控制结构体
typedef struct {
    interpolation_type_t type;     // 插补类型
    interpolation_state_t state;   // 插补状态
    
    union {
        line_interp_t line;        // 直线插补参数
        arc_interp_t arc;          // 圆弧插补参数
    } params;
    
    // 当前位置
    point_t current_pos;
    point_t target_pos;
    
    // 时间参数
    uint32_t interpolation_period_ms;  // 插补周期 (毫秒)
    uint32_t last_update_time;         // 上次更新时间
    
    // 关联的电机
    stepper_motor_t *motor_x;      // X轴电机
    stepper_motor_t *motor_y;      // Y轴电机
    
    // 回调函数
    void (*on_step_callback)(point_t current, point_t target);  // 每步回调
    void (*on_finish_callback)(void);                           // 完成回调
    
} gimbal_interpolation_t;

// 全局变量声明
extern gimbal_interpolation_t gimbal_interp;

// 函数声明

/**
 * @brief 初始化二维云台插补模块
 * @param interp 插补控制结构体指针
 * @param motor_x X轴步进电机指针
 * @param motor_y Y轴步进电机指针
 * @param period_ms 插补周期(毫秒)
 * @return 0:成功, -1:失败 
 */
int gimbal_interpolation_init(gimbal_interpolation_t *interp, 
                             stepper_motor_t *motor_x, 
                             stepper_motor_t *motor_y,
                             uint32_t period_ms);

/**
 * @brief 清理插补系统资源
 * @param interp 插补控制结构体指针
 * @return 0:成功, -1:失败
 */
int gimbal_interpolation_deinit(gimbal_interpolation_t *interp);

/**
 * @brief 设置直线插补
 * @param interp 插补控制结构体指针
 * @param start_x 起始X坐标
 * @param start_y 起始Y坐标  
 * @param end_x 终点X坐标
 * @param end_y 终点Y坐标
 * @param feed_rate 进给速度(度/秒)
 * @return 0:成功, -1:失败
 */
int gimbal_set_line_interpolation(gimbal_interpolation_t *interp,
                                 float start_x, float start_y,
                                 float end_x, float end_y,
                                 float feed_rate);

/**
 * @brief 设置圆弧插补
 * @param interp 插补控制结构体指针
 * @param start_x 起始X坐标
 * @param start_y 起始Y坐标
 * @param end_x 终点X坐标  
 * @param end_y 终点Y坐标
 * @param center_x 圆心X坐标
 * @param center_y 圆心Y坐标
 * @param clockwise 是否顺时针
 * @param feed_rate 进给速度(度/秒)
 * @return 0:成功, -1:失败
 */
int gimbal_set_arc_interpolation(gimbal_interpolation_t *interp,
                                float start_x, float start_y,
                                float end_x, float end_y,
                                float center_x, float center_y,
                                bool clockwise,
                                float feed_rate);

/**
 * @brief 开始插补运动
 * @param interp 插补控制结构体指针
 * @return 0:成功, -1:失败
 */
int gimbal_start_interpolation(gimbal_interpolation_t *interp);

/**
 * @brief 停止插补运动
 * @param interp 插补控制结构体指针
 * @return 0:成功, -1:失败
 */
int gimbal_stop_interpolation(gimbal_interpolation_t *interp);

/**
 * @brief 插补任务处理函数(需要周期性调用)
 * @param interp 插补控制结构体指针
 */
void gimbal_interpolation_task(gimbal_interpolation_t *interp);

/**
 * @brief 获取插补状态
 * @param interp 插补控制结构体指针
 * @return 插补状态
 */
interpolation_state_t gimbal_get_interpolation_state(gimbal_interpolation_t *interp);

/**
 * @brief 获取当前位置
 * @param interp 插补控制结构体指针
 * @param pos 位置指针
 */
void gimbal_get_current_position(gimbal_interpolation_t *interp, point_t *pos);

/**
 * @brief 设置插补完成回调函数
 * @param interp 插补控制结构体指针
 * @param callback 回调函数指针
 */
void gimbal_set_finish_callback(gimbal_interpolation_t *interp, void (*callback)(void));

/**
 * @brief 设置插补步进回调函数
 * @param interp 插补控制结构体指针
 * @param callback 回调函数指针
 */
void gimbal_set_step_callback(gimbal_interpolation_t *interp, 
                             void (*callback)(point_t current, point_t target));

/**
 * @brief 检查是否运动完成
 * @param interp 插补控制结构体指针
 * @return true:完成, false:未完成
 */
bool gimbal_is_motion_finished(gimbal_interpolation_t *interp);

// 数学辅助函数
float normalize_angle(float angle);
float calculate_distance(point_t p1, point_t p2);
float calculate_angle(point_t center, point_t point);

// 高级应用函数

/**
 * @brief 执行G01直线插补命令
 * @param end_x 终点X坐标
 * @param end_y 终点Y坐标
 * @param feed_rate 进给速度
 * @return 0:成功, -1:失败
 */
int gimbal_g01_line(float end_x, float end_y, float feed_rate);

/**
 * @brief 执行G02顺时针圆弧插补命令
 * @param end_x 终点X坐标
 * @param end_y 终点Y坐标
 * @param center_x 圆心X坐标
 * @param center_y 圆心Y坐标
 * @param feed_rate 进给速度
 * @return 0:成功, -1:失败
 */
int gimbal_g02_arc_cw(float end_x, float end_y, float center_x, float center_y, float feed_rate);

/**
 * @brief 执行G03逆时针圆弧插补命令
 * @param end_x 终点X坐标
 * @param end_y 终点Y坐标
 * @param center_x 圆心X坐标
 * @param center_y 圆心Y坐标
 * @param feed_rate 进给速度
 * @return 0:成功, -1:失败
 */
int gimbal_g03_arc_ccw(float end_x, float end_y, float center_x, float center_y, float feed_rate);

/**
 * @brief 等待运动完成
 * @param timeout_ms 超时时间(毫秒), 0表示无限等待
 * @return 0:成功完成, -1:超时
 */
int gimbal_wait_motion_finish(uint32_t timeout_ms);

/**
 * @brief 启动插补定时器
 * @return 0:成功, -1:失败
 */
int gimbal_start_interpolation_timer(void);

/**
 * @brief 停止插补定时器  
 * @return 0:成功, -1:失败
 */
int gimbal_stop_interpolation_timer(void);

/**
 * @brief 中断安全的电机位置控制函数 - 不使用互斥锁
 * @param motor 电机指针
 * @param dir 方向 (0或1)
 * @param vel 速度
 * @param acc 加速度
 * @param clk 脉冲数
 * @param raF 相对/绝对标志
 * @param snF 同步标志
 * @return 0:成功, -1:失败
 */
int gimbal_emm_pos_control_isr_safe(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_INTERPOLATION_H
