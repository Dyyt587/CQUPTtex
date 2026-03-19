#include "gimbal_interpolation.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include <math.h>
#include <string.h>

#define DBG_TAG "gimbal.interp"
#define DBG_LVL DBG_LOG

// 全局插补控制结构体
gimbal_interpolation_t gimbal_interp;

// 数学常量
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ANGLE_EPSILON 1e-6  // 角度比较精度
#define POSITION_EPSILON 0.01  // 位置比较精度

// 外部变量声明
extern uint16_t motor_vel;
extern int motor_acc;

// 外部函数声明
extern void emm_transmit(uint8_t *data, uint8_t len);

/**
 * @brief 中断安全的电机位置控制函数 - 直接发送命令，不使用互斥锁
 */
int gimbal_emm_pos_control_isr_safe(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    if (!motor) {
        return -1;
    }
    
    uint8_t cmd[16] = {0};
    
		if(dir<0)
		{
			LOG_D("dir<0");
		}
    // 装载命令 - 与Emm_V5_Pos_Control相同的协议格式
    cmd[0] = motor->stepper_motor_id;   // 地址
    //cmd[0] = 2;   // 地址
    cmd[1] = 0xFD;                      // 功能码
    cmd[2] = dir;                       // 方向
    cmd[3] = (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
    cmd[5] = acc;                       // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                      // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                      // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                     // 校验字节
    
    // 直接发送命令，不使用互斥锁
    emm_transmit(cmd, 13);
    
    // 更新电机目标角度（用于状态跟踪）
    if (raF) {
        // 绝对位置
        motor->stepper_motor_target_angle = (float)clk * 360.0f / 3200.0f;
    } else {
        // 相对位置
        float delta_angle = (float)clk * 360.0f / 3200.0f;
        if (dir == 0) {
            delta_angle = -delta_angle;
        }
        motor->stepper_motor_target_angle = motor->stepper_motor_angle + delta_angle;
    }
    
    return 0;
}

/**
 * @brief 初始化二维云台插补模块
 */
int gimbal_interpolation_init(gimbal_interpolation_t *interp, 
                             stepper_motor_t *motor_x, 
                             stepper_motor_t *motor_y,
                             uint32_t period_ms)
{
	static int is_init=0;
	
	if(is_init==1){
		return 0;
	}
	is_init =1;
    if (!interp || !motor_x || !motor_y) {
        LOG_E("Invalid parameters for interpolation init");
        return -1;
    }
    
    // 清零结构体
    memset(interp, 0, sizeof(gimbal_interpolation_t));
    
    // 设置基本参数
    interp->motor_x = motor_x;
    interp->motor_y = motor_y;
    interp->interpolation_period_ms = period_ms;
    interp->state = INTERP_STATE_IDLE;
    
    // 初始化当前位置为电机当前角度
    interp->current_pos.x = motor_x->stepper_motor_angle;
    interp->current_pos.y = motor_y->stepper_motor_angle;
    
    LOG_I("Gimbal interpolation initialized, period: %d ms", period_ms);
    LOG_I("Current position: X=%.3f, Y=%.3f", interp->current_pos.x, interp->current_pos.y);
    
    return 0;
}

/**
 * @brief 设置直线插补参数
 */
int gimbal_set_line_interpolation(gimbal_interpolation_t *interp,
                                 float start_x, float start_y,
                                 float end_x, float end_y,
                                 float feed_rate)
{
    if (!interp) {
        LOG_E("Invalid interpolation pointer");
        return -1;
    }
    
    if (interp->state == INTERP_STATE_RUNNING) {
        LOG_W("Interpolation is running, stop it first");
        return -1;
    }
    
    if (feed_rate <= 0) {
        LOG_E("Invalid feed rate: %.3f", feed_rate);
        return -1;
    }
    
    // 设置插补类型
    interp->type = INTERP_TYPE_LINE;
    
    // 设置直线插补参数
    line_interp_t *line = &interp->params.line;
    line->start_pos.x = start_x;
    line->start_pos.y = start_y;
    line->end_pos.x = end_x;
    line->end_pos.y = end_y;
    line->feed_rate = feed_rate;
    
    // 计算距离和步数
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float distance = sqrtf(dx * dx + dy * dy);
    
    if (distance < POSITION_EPSILON) {
        LOG_W("Distance too small: %.6f", distance);
        interp->state = INTERP_STATE_FINISHED;
        return 0;
    }
    
    // 计算总运行时间和步数
    float total_time = distance / feed_rate;  // 秒
    line->total_steps = (uint32_t)(total_time * 1000 / interp->interpolation_period_ms);
    
    if (line->total_steps == 0) {
        line->total_steps = 1;
    }
    
    line->current_step = 0;
    
    // 设置起始位置
    interp->current_pos = line->start_pos;
    interp->target_pos = line->end_pos;
    
    LOG_I("Line interpolation set:");
    LOG_I("  Start: (%.3f, %.3f)", start_x, start_y);
    LOG_I("  End: (%.3f, %.3f)", end_x, end_y);
    LOG_I("  Distance: %.3f", distance);
    LOG_I("  Feed rate: %.3f deg/s", feed_rate);
    LOG_I("  Total steps: %d", line->total_steps);
    LOG_I("  Step time: %d ms", interp->interpolation_period_ms);
    
    interp->state = INTERP_STATE_IDLE;
    return 0;
}

/**
 * @brief 开始插补运动
 */
int gimbal_start_interpolation(gimbal_interpolation_t *interp)
{
    if (!interp) {
        LOG_E("Invalid interpolation pointer");
        return -1;
    }
    
    if (interp->state == INTERP_STATE_RUNNING) {
        LOG_W("Interpolation already running");
        return 0;
    }
    
    if (interp->type == INTERP_TYPE_LINE) {
        line_interp_t *line = &interp->params.line;
        if (line->total_steps == 0) {
            LOG_E("Line interpolation not configured");
            return -1;
        }
        
        line->current_step = 0;
        interp->current_pos = line->start_pos;
    }
    
    interp->state = INTERP_STATE_RUNNING;
    interp->last_update_time = rt_tick_get();
    
    LOG_I("Interpolation started");
    return 0;
}

/**
 * @brief 停止插补运动
 */
int gimbal_stop_interpolation(gimbal_interpolation_t *interp)
{
    if (!interp) {
        LOG_E("Invalid interpolation pointer");
        return -1;
    }
    
    if (interp->state == INTERP_STATE_RUNNING) {
        interp->state = INTERP_STATE_IDLE;
        
        // 注意：这里不能直接调用Emm_V5_Stop_Now，因为可能在中断中调用
        // 可以发送停止命令到队列，或者设置标志位让线程处理
        LOG_I("Interpolation stop requested");
    }
    
    return 0;
}

/**
 * @brief 直线插补计算
 */
static void calculate_line_interpolation(gimbal_interpolation_t *interp, point_t *next_pos)
{
    line_interp_t *line = &interp->params.line;
    
    if (line->current_step >= line->total_steps) {
        // 插补完成
        *next_pos = line->end_pos;
        interp->state = INTERP_STATE_FINISHED;
        return;
    }
    
    // 计算插补比例
    float ratio = (float)line->current_step / (float)line->total_steps;
    
    // 线性插补计算
    next_pos->x = line->start_pos.x + ratio * (line->end_pos.x - line->start_pos.x);
    next_pos->y = line->start_pos.y + ratio * (line->end_pos.y - line->start_pos.y);
    
    line->current_step++;
}

/**
 * @brief 控制电机移动到指定位置 - 直接在定时器中调用，使用中断安全函数
 */
static void move_motors_to_position(gimbal_interpolation_t *interp, point_t target)
{
    // 计算相对于当前位置的移动量
    float delta_x = target.x - interp->motor_x->stepper_motor_angle;
    float delta_y = target.y - interp->motor_y->stepper_motor_angle;
    
    // 如果移动量很小，跳过
    if (fabsf(delta_x) < POSITION_EPSILON && fabsf(delta_y) < POSITION_EPSILON) {
        return;
    }
    
    // 设置电机目标角度
    interp->motor_x->stepper_motor_target_angle = target.x;
    interp->motor_y->stepper_motor_target_angle = target.y;
    
    // 计算脉冲数 (假设3200脉冲/圈，360度/圈)
    uint32_t pulses_x = (uint32_t)((delta_x) * 6400.0f / 360.0f);
    uint32_t pulses_y = (uint32_t)((delta_y) * 6400.0f / 360.0f);
    
		extern int left_stepper_pulse;
		extern int right_stepper_pulse;
		
		left_stepper_pulse = pulses_x;
		right_stepper_pulse = pulses_y;
		LOG_D("deltax:%d deltay:%d",left_stepper_pulse,right_stepper_pulse);
}

/**
 * @brief 插补任务处理函数
 */
void gimbal_interpolation_task(gimbal_interpolation_t *interp)
{
    if (!interp || interp->state != INTERP_STATE_RUNNING) {
        return;
    }
    
    uint32_t current_time = rt_tick_get();
    uint32_t elapsed_time = current_time - interp->last_update_time;
    
    // 检查是否到了插补周期
    if (elapsed_time < (interp->interpolation_period_ms * RT_TICK_PER_SECOND / 1000)) {
        return;
    }
    
    point_t next_pos;
    
    // 根据插补类型计算下一个位置
    switch (interp->type) {
        case INTERP_TYPE_LINE:
            calculate_line_interpolation(interp, &next_pos);
            break;
            
        case INTERP_TYPE_ARC_CW:
        case INTERP_TYPE_ARC_CCW:
            // 圆弧插补将在后续实现
            LOG_W("Arc interpolation not implemented yet");
            interp->state = INTERP_STATE_ERROR;
            return;
            
        default:
            LOG_E("Unknown interpolation type: %d", interp->type);
            interp->state = INTERP_STATE_ERROR;
            return;
    }
    
    // 控制电机移动
    move_motors_to_position(interp, next_pos);
    
    // 更新当前位置
    interp->current_pos = next_pos;
    
    // 调用步进回调函数
    if (interp->on_step_callback) {
        interp->on_step_callback(interp->current_pos, next_pos);
    }
    
    // 检查是否完成
    if (interp->state == INTERP_STATE_FINISHED) {
        LOG_I("Interpolation finished at (%.3f, %.3f)", 
              interp->current_pos.x, interp->current_pos.y);
        
        // 调用完成回调函数
        if (interp->on_finish_callback) {
            interp->on_finish_callback();
        }
    }
    
    interp->last_update_time = current_time;
}

/**
 * @brief 获取插补状态
 */
interpolation_state_t gimbal_get_interpolation_state(gimbal_interpolation_t *interp)
{
    if (!interp) {
        return INTERP_STATE_ERROR;
    }
    return interp->state;
}

/**
 * @brief 获取当前位置
 */
void gimbal_get_current_position(gimbal_interpolation_t *interp, point_t *pos)
{
    if (!interp || !pos) {
        return;
    }
    *pos = interp->current_pos;
}

/**
 * @brief 设置插补完成回调函数
 */
void gimbal_set_finish_callback(gimbal_interpolation_t *interp, void (*callback)(void))
{
    if (interp) {
        interp->on_finish_callback = callback;
    }
}

/**
 * @brief 设置插补步进回调函数
 */
void gimbal_set_step_callback(gimbal_interpolation_t *interp, 
                             void (*callback)(point_t current, point_t target))
{
    if (interp) {
        interp->on_step_callback = callback;
    }
}

/**
 * @brief 检查是否运动完成
 */
bool gimbal_is_motion_finished(gimbal_interpolation_t *interp)
{
    if (!interp) {
        return true;
    }
    return (interp->state == INTERP_STATE_FINISHED || interp->state == INTERP_STATE_IDLE);
}

/**
 * @brief 数学辅助函数 - 计算两点距离
 */
float calculate_distance(point_t p1, point_t p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 数学辅助函数 - 角度归一化
 */
float normalize_angle(float angle)
{
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

/**
 * @brief 执行G01直线插补命令
 */
int gimbal_g01_line(float end_x, float end_y, float feed_rate)
{
    point_t current_pos;
    gimbal_get_current_position(&gimbal_interp, &current_pos);
    
    int ret = gimbal_set_line_interpolation(&gimbal_interp,
                                           current_pos.x, current_pos.y,
                                           end_x, end_y,
                                           feed_rate);
    if (ret != 0) {
        return ret;
    }
    
    return gimbal_start_interpolation(&gimbal_interp);
}

/**
 * @brief 等待运动完成
 */
int gimbal_wait_motion_finish(uint32_t timeout_ms)
{
    uint32_t start_time = rt_tick_get();
    uint32_t timeout_ticks = timeout_ms * RT_TICK_PER_SECOND / 1000;
    
    while (!gimbal_is_motion_finished(&gimbal_interp)) {
        rt_thread_mdelay(10);
        
        if (timeout_ms > 0) {
            uint32_t elapsed = rt_tick_get() - start_time;
            if (elapsed > timeout_ticks) {
                LOG_W("Motion timeout after %d ms", timeout_ms);
                return -1;
            }
        }
    }
    
    return 0;
}

// RT-Thread 定时器回调函数，用于周期性执行插补任务
static rt_timer_t interp_timer = RT_NULL;

static void interpolation_timer_callback(void *parameter)
{
    gimbal_interpolation_task(&gimbal_interp);
}

/**
 * @brief 启动插补定时器
 */
int gimbal_start_interpolation_timer(void)
{
    if (interp_timer == RT_NULL) {
        interp_timer = rt_timer_create("gimbal_interp",
                                      interpolation_timer_callback,
                                      RT_NULL,
                                      gimbal_interp.interpolation_period_ms * RT_TICK_PER_SECOND / 1000,
                                      RT_TIMER_FLAG_PERIODIC);
        if (interp_timer == RT_NULL) {
            LOG_E("Failed to create interpolation timer");
            return -1;
        }
    }
    
    rt_timer_start(interp_timer);
    LOG_I("Interpolation timer started");
    return 0;
}

/**
 * @brief 停止插补定时器
 */
int gimbal_stop_interpolation_timer(void)
{
    if (interp_timer != RT_NULL) {
        rt_timer_stop(interp_timer);
        LOG_I("Interpolation timer stopped");
    }
    return 0;
}

/**
 * @brief 清理插补系统资源
 */
int gimbal_interpolation_deinit(gimbal_interpolation_t *interp)
{
    if (!interp) {
        return -1;
    }
    
    // 停止定时器
    gimbal_stop_interpolation_timer();
    
    // 停止插补
    gimbal_stop_interpolation(interp);
    
    LOG_I("Gimbal interpolation deinitialized");
    return 0;
}
