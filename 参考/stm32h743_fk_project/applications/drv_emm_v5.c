#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#define DBG_TAG "drv.step_motor"
#define DBG_LVL DBG_LOG
#include "drv_emm_v5.h"
//#include "drv_corexy.h"
#include "math.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))
rt_mutex_t mutex_step = RT_NULL; // 互斥锁
rt_sem_t emm_rx_sem = RT_NULL;
rt_device_t Emm_serial1 = RT_NULL;
int Emm_rx_size = 0;
uint8_t Emm_rx_buf[64] = {0};

//stepper_motor_t stepper_right;
//stepper_motor_t stepper_left;

stepper_motor_t stepper_motor_1;
stepper_motor_t stepper_motor_2;
stepper_motor_t stepper_motor_3;
stepper_motor_t stepper_motor_4;

int motor_acc = 0x0;
uint16_t motor_vel = 5;
// #define motor_acc 0xff
// #define motor_vel 150
//v=0.1 m/s
int gap_time = 10;




//	extern corexy_t corexy;




void emm_transmit(uint8_t *data, uint8_t len)
{
    rt_device_write(Emm_serial1, 0, data, len);
}

int emm_get_cmd_receive_size(uint8_t cmd)
{
    switch (cmd)
    {
        case 0x35: // v
            return 6;
        case 0x36: // p
            return 8;
        default:
            return 4;
    }
}

void emm_cmd_ana(stepper_motor_t *motor, uint8_t cmd, uint8_t *buf)
{   
    //LOG_D("cmd:%d",cmd);
    switch (cmd)
    {
    case 0x35: // v
        if (Emm_rx_buf[1] == 0x35)
        {

            // 拼接成uint16_t类型数据
            uint16_t vel = (uint16_t)(((uint16_t)Emm_rx_buf[3] << 8) |
                                      ((uint16_t)Emm_rx_buf[4] << 0));
            motor->stepper_motor_speed = vel;

            // 符号
            if (Emm_rx_buf[2])
            {
                motor->stepper_motor_speed = -motor->stepper_motor_speed;
            }
       //     LOG_D("emm_cmd_ana v:%d", motor->stepper_motor_speed);
            return;
        }
        else
            return;
    case 0x36: // p
        if(Emm_rx_buf[1] == 0x36)
        {
                //获取绝对值信息
                float pos = (uint32_t)(
                                ((uint32_t)Emm_rx_buf[3] << 24)    |
                                ((uint32_t)Emm_rx_buf[4] << 16)    |
                                ((uint32_t)Emm_rx_buf[5] << 8)     |
                                ((uint32_t)Emm_rx_buf[6] << 0)
                                );
                // 转换为角度
                motor->stepper_motor_angle = (float)pos * 360.0f / 65536.0f;

                // 修正电机正反位置
                if(Emm_rx_buf[2])
                {
                    motor->stepper_motor_angle = -motor->stepper_motor_angle;
                }
             //   LOG_I("stepperMotor_%d_Cur_Pos: %f\n",motor->stepper_motor_id , motor->stepper_motor_angle);     
                return;             
        }
        else
            return;
    default:
        if (Emm_rx_buf[3] != 0x6B)
        for(int i=0;i<4;i++)
        {
           LOG_D("Emm_rx_buf[%d]:%x",i,Emm_rx_buf[i]);
        }
				

				rt_thread_mdelay(20);
				
            //LOG_W("emm_cmd answer nuknown");
        return;
    }
}
void emm_wait_for_ack(stepper_motor_t *motor, uint8_t cmd)
{
    uint8_t will_rx_size = emm_get_cmd_receive_size(cmd);
    // LOG_D("will_rx_size:%d",will_rx_size);
    uint8_t Emm_rx_index = 0;
    uint8_t ch = 0;
    while (1)
    {
        while (rt_device_read(Emm_serial1, -1, &ch, 1) != 1)	//当没有接收到数据的时候一直循环
        {
					rt_err_t ret = rt_sem_take(emm_rx_sem, 10);	//当10ms没有接收到数据的时候，释放信号量，强制退出
            if (ret != RT_EOK)
            {
							rt_memset(Emm_rx_buf,0,sizeof(Emm_rx_buf));
							LOG_E("emm_wait_for_ack timeout cmd :%d",cmd);
                rt_mutex_release(mutex_step);
                return;
            }
            else
            {

            }
        }
        // 获取到字符
        Emm_rx_buf[Emm_rx_index++] = ch;
        if (Emm_rx_index == will_rx_size)
        {
            emm_cmd_ana(motor, cmd, Emm_rx_buf);//进行数据解析
            rt_mutex_release(mutex_step);	//释放互斥量
            return;
        }
    }
    
    rt_mutex_release(mutex_step);
}
/**
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_CurPos_To_Zero(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x0A; // 功能码
    cmd[2] = 0x6D; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_Clog_Pro(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x0E; // 功能码
    cmd[2] = 0x52; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Read_Sys_Params(stepper_motor_t *motor, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[i] = motor->stepper_motor_id;
    ++i; // 地址

    switch (s) // 功能码
    {
    case S_VER:
        cmd[i] = 0x1F;
        ++i;
        break;
    case S_RL:
        cmd[i] = 0x20;
        ++i;
        break;
    case S_PID:
        cmd[i] = 0x21;
        ++i;
        break;
    case S_VBUS:
        cmd[i] = 0x24;
        ++i;
        break;
    case S_CPHA:
        cmd[i] = 0x27;
        ++i;
        break;
    case S_ENCL:
        cmd[i] = 0x31;
        ++i;
        break;
    case S_TPOS:
        cmd[i] = 0x33;
        ++i;
        break;
    case S_VEL:
        cmd[i] = 0x35;
        ++i;
        break;
    case S_CPOS:
        cmd[i] = 0x36;
        ++i;
        break;
    case S_PERR:
        cmd[i] = 0x37;
        ++i;
        break;
    case S_FLAG:
        cmd[i] = 0x3A;
        ++i;
        break;
    case S_ORG:
        cmd[i] = 0x3B;
        ++i;
        break;
    case S_Conf:
        cmd[i] = 0x42;
        ++i;
        cmd[i] = 0x6C;
        ++i;
        break;
    case S_State:
        cmd[i] = 0x43;
        ++i;
        cmd[i] = 0x7A;
        ++i;
        break;
    default:
        break;
    }

    cmd[i] = 0x6B;
    ++i; // 校验字节

    // 发送命令
    emm_transmit(cmd, i);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Modify_Ctrl_Mode(stepper_motor_t *motor, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id;      // 地址
    cmd[1] = 0x46;      // 功能码
    cmd[2] = 0x69;      // 辅助码
    cmd[3] = svF;       // 是否存储标志，false为不存储，true为存储
    cmd[4] = ctrl_mode; // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
    cmd[5] = 0x6B;      // 校验字节

    // 发送命令
    emm_transmit(cmd, 6);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_En_Control(stepper_motor_t *motor, bool state, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id;           // 地址
    cmd[1] = 0xF3;           // 功能码
    cmd[2] = 0xAB;           // 辅助码
    cmd[3] = (uint8_t)state; // 使能状态
    cmd[4] = snF;            // 多机同步运动标志
    cmd[5] = 0x6B;           // 校验字节

    // 发送命令
    emm_transmit(cmd, 6);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Vel_Control(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

		if(dir==0)	//假设dir=0为正转
		{
			motor->stepper_motor_target_speed=vel;
		}
		else{
			motor->stepper_motor_target_speed=vel;
			motor->stepper_motor_target_speed*=-1;
		}
    // 装载命令
    cmd[0] = motor->stepper_motor_id;                // 地址
    cmd[1] = 0xF6;                // 功能码
    cmd[2] = dir;                 // 方向
    cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
    cmd[5] = acc;                 // 加速度，注意：0是直接启动
    cmd[6] = snF;                 // 多机同步运动标志
    cmd[7] = 0x6B;                // 校验字节

    // 发送命令
    emm_transmit(cmd, 8);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_Control(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);
		
		if(dir==0)	//假设dir=0为正转
		{
			motor->stepper_motor_target_speed=vel;
		}
		else{
			motor->stepper_motor_target_speed=vel;
			motor->stepper_motor_target_speed*=-1;
		}
		
    // 装载命令
    cmd[0] = motor->stepper_motor_id;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 发送命令
    emm_transmit(cmd, 13);
   emm_wait_for_ack(motor,cmd[1]);


}


/**
 * @brief    立即停止（所有控制模式都通用）
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Stop_Now(stepper_motor_t *motor, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0xFE; // 功能码
    cmd[2] = 0x98; // 辅助码
    cmd[3] = snF;  // 多机同步运动标志
    cmd[4] = 0x6B; // 校验字节

    // 发送命令
    emm_transmit(cmd, 5);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Synchronous_motion(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);
//motor->stepper_motor_id
    // 装载命令
    cmd[0] = 0; // 地址
    cmd[1] = 0xFF; // 功能码
    cmd[2] = 0x66; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    设置单圈回零的零点位置
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Set_O(stepper_motor_t *motor, bool svF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x93; // 功能码
    cmd[2] = 0x88; // 辅助码
    cmd[3] = svF;  // 是否存储标志，false为不存储，true为存储
    cmd[4] = 0x6B; // 校验字节

    // 发送命令
    emm_transmit(cmd, 5);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    修改回零参数
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    o_dir  ：回零方向，0为CW，其余值为CCW
 * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
 * @param    o_tm   ：回零超时时间，单位：毫秒
 * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param    potF   ：上电自动触发回零，false为不使能，true为使能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Modify_Params(stepper_motor_t *motor, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id;                    // 地址
    cmd[1] = 0x4C;                    // 功能码
    cmd[2] = 0xAE;                    // 辅助码
    cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
    cmd[4] = o_mode;                  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] = o_dir;                   // 回零方向
    cmd[6] = (uint8_t)(o_vel >> 8);   // 回零速度(RPM)高8位字节
    cmd[7] = (uint8_t)(o_vel >> 0);   // 回零速度(RPM)低8位字节
    cmd[8] = (uint8_t)(o_tm >> 24);   // 回零超时时间(bit24 - bit31)
    cmd[9] = (uint8_t)(o_tm >> 16);   // 回零超时时间(bit16 - bit23)
    cmd[10] = (uint8_t)(o_tm >> 8);   // 回零超时时间(bit8  - bit15)
    cmd[11] = (uint8_t)(o_tm >> 0);   // 回零超时时间(bit0  - bit7 )
    cmd[12] = (uint8_t)(sl_vel >> 8); // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] = (uint8_t)(sl_vel >> 0); // 无限位碰撞回零检测转速(RPM)低8位字节
    cmd[14] = (uint8_t)(sl_ma >> 8);  // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] = (uint8_t)(sl_ma >> 0);  // 无限位碰撞回零检测电流(Ma)低8位字节
    cmd[16] = (uint8_t)(sl_ms >> 8);  // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] = (uint8_t)(sl_ms >> 0);  // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] = potF;                   // 上电自动触发回零，false为不使能，true为使能
    cmd[19] = 0x6B;                   // 校验字节

    // 发送命令
    emm_transmit(cmd, 20);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    触发回零
 * @param    addr   ：电机地址
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Trigger_Return(stepper_motor_t *motor, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id;   // 地址
    cmd[1] = 0x9A;   // 功能码
    cmd[2] = o_mode; // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[3] = snF;    // 多机同步运动标志，false为不启用，true为启用
    cmd[4] = 0x6B;   // 校验字节

    // 发送命令
    emm_transmit(cmd, 5);
    emm_wait_for_ack(motor,cmd[1]);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Interrupt(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x9C; // 功能码
    cmd[2] = 0x48; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor,cmd[1]);
}
rt_err_t emm_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    Emm_rx_size = size;
    rt_sem_release(emm_rx_sem);
    return RT_EOK;
}


//void Emm_V5_Pos_moveok(void)
//{
//    static int time=0;
//    while( 1 )
//    {
//        time++;

//        corexy.present.x = -(float)(stepper_right.stepper_motor_angle*142.22f + stepper_left.stepper_motor_angle*142.22f) * 0.04f / (float)(256*200) / 2;
//        corexy.present.y = -(float)(stepper_right.stepper_motor_angle*142.22f - stepper_left.stepper_motor_angle*142.22f) * 0.04f / (float)(256*200) / 2;
//        float error_x = fabs(corexy.target.x - corexy.present.x);
//        float error_y = fabs(corexy.target.y - corexy.present.y);
//        if(fabs(corexy.target.x - corexy.present.x) < 0.005f  &&  fabs(corexy.target.y - corexy.present.y) < 0.005f )
//        {
//            // real_corexy.x = corexy.x;
//            // real_corexy.y = corexy.y;
//            return;
//        }
//        
//       static int time=0;

//        if(time++%50==0)
//        {
//        // LOG_D("eror_x %f,error_y %f",error_x,error_y);
//        }
//        if(time == 500) 
//        {
//            time = 0;
//            LOG_E("stepper_motor_move_timeout");
//            break;
//        }

//        rt_thread_mdelay(1);
//    }
//}


int left_stepper_pulse=0;
int right_stepper_pulse=0;
int left_stepper_pulse_abs=0;
int right_stepper_pulse_abs=0;
void drv_emm_v5_entry(void *t)
{
	

    //rt_thread_mdelay(2000); //等待步进上电
    /* 查找系统中的串口设备 */
    Emm_serial1 = rt_device_find("uart3");
	if(Emm_serial1==RT_NULL)return ;
    char ch = 0;
    /* 初始化信号量 */
    emm_rx_sem = rt_sem_create("emm_rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 以阻塞接收及轮询发送模式打开串口设备 */
    rt_device_open(Emm_serial1, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    if (rt_device_set_rx_indicate(Emm_serial1, emm_uart_rx_ind) != RT_EOK)
    {
        LOG_E("uart8 set rx indicate failed");
        return ;
    }

//    stepper_right.stepper_motor_id = 1;
//    stepper_left.stepper_motor_id = 2;
		stepper_motor_1.stepper_motor_id=1;
		stepper_motor_2.stepper_motor_id=2;
		stepper_motor_3.stepper_motor_id=3;
		stepper_motor_4.stepper_motor_id=4;
    
#define Origin_vel 100
#define timeout 30000
//这里的堵转速度直接给最大值 就不用管这个参数了
#define sl_vel 100 
//用大电流 小时间来60
#define sl_ma 40
		//时间
#define sl_ms 100
    //回零调参 电机有存储的 没必要每次都重新设置
    /*
     * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    o_dir  ：回零方向，0为CW，其余值为CCW
 * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
 * @param    o_tm   ：回零超时时间，单位：毫秒
 * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param    potF   ：上电自动触发回零，false为不使能，true为使能
 * */
    
//     Emm_V5_Origin_Modify_Params(&stepper_right, 1, 2, 1, Origin_vel, timeout, sl_vel, sl_ma, sl_ms, 0);//参数4是方向   倒数第三个参数是电流值   这是控制大臂的电机（螺丝很长的一边）  方向1 是控制往上抬
//     Emm_V5_Origin_Modify_Params(&stepper_left, 1, 2, 1, Origin_vel, timeout, sl_vel, sl_ma, sl_ms, 0);//参数4是方向 倒数第三个参数是电流值    这是控制小臂的电机（多一个件的一边）  第四个参数 方向0 是控制往上抬
//  
//     rt_thread_mdelay(100); //设置参数之后需要延时！！！！！！！！！延时等待闭环步进参数设置完成（写入flash）
   

//		      Emm_V5_En_Control(&stepper_left, 0, 0);//一个一个回零
//     //Emm_V5_En_Control(&stepper_right, 1, 0);//一个一个回零
//     Emm_V5_Origin_Trigger_Return(&stepper_right, 2, 0);
//     rt_thread_mdelay(5000);

//     Emm_V5_En_Control(&stepper_left, 1, 0);//一个一个回零
//     Emm_V5_En_Control(&stepper_right, 0, 0);//一个一个回零
//     Emm_V5_Origin_Trigger_Return(&stepper_left, 2, 0);
//     rt_thread_mdelay(5000);

    // Emm_V5_Reset_CurPos_To_Zero(&stepper_right);
    // Emm_V5_Reset_CurPos_To_Zero(&stepper_left);
    
		 Emm_V5_En_Control(&stepper_motor_1, 0, 0);
     Emm_V5_En_Control(&stepper_motor_2, 0, 0);
//		 Emm_V5_En_Control(&stepper_motor_3, 0, 0);
//     Emm_V5_En_Control(&stepper_motor_4, 0, 0);
		 
		 Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_1);
     Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_2);
//		 Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_3);
//     Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_4);
		 
     Emm_V5_En_Control(&stepper_motor_1, 1, 0);
     Emm_V5_En_Control(&stepper_motor_2, 1, 0);
//		 Emm_V5_En_Control(&stepper_motor_3, 1, 0);
//     Emm_V5_En_Control(&stepper_motor_4, 1, 0);
    // corexy.x=0;
    // corexy.y=0;


    int x_pulse = 0;
    int y_pulse = 0;

    
    while (1)
    {
        //1.8*256*200 对应 0.04m
        // #define laser_visio_offsety  0.027
        // #define laser_visio_offsetx  0.002

        // float laser_visio_offsety = 0.027;
        // float laser_visio_offsetx = 0.002;
        // x_pulse = (corexy.x+laser_visio_offsetx) * (float)(256*200) / 0.04f ;
        // y_pulse = -(corexy.y-laser_visio_offsety) * (float)(256*200) / 0.04f ;
//        x_pulse = corexy.target.x * (float)(256*200) / 0.04f ;
//        y_pulse = -corexy.target.y * (float)(256*200) / 0.04f ;

//        left_stepper_pulse = x_pulse - y_pulse;
//        right_stepper_pulse = x_pulse + y_pulse;

			

//					Emm_V5_Vel_Control(&stepper_motor_1,1,100,50,0);
//					Emm_V5_Vel_Control(&stepper_motor_2,1,100,50,0);
//					Emm_V5_Vel_Control(&stepper_motor_3,1,100,50,0);
//					Emm_V5_Vel_Control(&stepper_motor_4,1,100,50,0);
//        static int time=0;
////        corexy.present.x = -(float)(stepper_right.stepper_motor_angle*142.22f + stepper_left.stepper_motor_angle*142.22f) * 0.04f / (float)(256*200) / 2;
////        corexy.present.y = -(float)(stepper_right.stepper_motor_angle*142.22f - stepper_left.stepper_motor_angle*142.22f) * 0.04f / (float)(256*200) / 2;
//        if(time++%100==0)
//        {
////            LOG_D("tar_corexy.x:%f,tar_corexy.y:%f",corexy.target.x,corexy.target.y);
////            LOG_D("real_corexy.x:%f,real_corexy.y:%f",corexy.present.x,corexy.present.y);
//					
//									LOG_D("angle is 1:%.1f,2:%.1f,3:%.1f,4:%.1f",stepper_motor_1.stepper_motor_angle,\
//				stepper_motor_2.stepper_motor_angle,stepper_motor_3.stepper_motor_angle,stepper_motor_4.stepper_motor_angle);
//					
//        }
//           			Emm_V5_Pos_Control(&stepper_motor_1,0,150,0,6400,true,1);
//					Emm_V5_Pos_Control(&stepper_motor_1,0,150,0,6400,true,0);
//					Emm_V5_Pos_Control(&stepper_motor_2,0,150,0,6400,true,0);
//					Emm_V5_Pos_Control(&stepper_motor_4,0,150,0,6400,true,1);
//        Emm_V5_Synchronous_motion(&stepper_motor_1);
        
        Emm_V5_Read_Sys_Params(&stepper_motor_1, S_CPOS);
        Emm_V5_Read_Sys_Params(&stepper_motor_2, S_CPOS);
//		  Emm_V5_Read_Sys_Params(&stepper_motor_3, S_CPOS);
//        Emm_V5_Read_Sys_Params(&stepper_motor_4, S_CPOS);
//        LOG_D("1:%.1f,2:%.1f,3:%.1f,4:%.1f",stepper_motor_1.stepper_motor_angle,\
//				stepper_motor_2.stepper_motor_angle,stepper_motor_3.stepper_motor_angle,stepper_motor_4.stepper_motor_angle);

//        Emm_V5_Read_Sys_Params(&stepper_motor_4, S_CPOS); 
        Emm_V5_Read_Sys_Params(&stepper_motor_1, S_VEL);
        Emm_V5_Read_Sys_Params(&stepper_motor_2, S_VEL);
//		Emm_V5_Read_Sys_Params(&stepper_motor_3, S_VEL);
//        Emm_V5_Read_Sys_Params(&stepper_motor_4, S_VEL); 
//        LOG_D("speed is 1:%.1f,2:%.1f,3:%.1f,4:%.1f",stepper_motor_1.stepper_motor_speed,\
//				stepper_motor_2.stepper_motor_speed,stepper_motor_3.stepper_motor_speed,stepper_motor_4.stepper_motor_speed);        
//        rt_thread_mdelay(5);//150有点震


			static float t=0;
			static float deltat=0.05;
 
			static float x,y;
			static float h=1;
			static float sign=1;
			static float xx=0.2;
			static float omg=8;
			
			if(x>xx)sign=1;
			if(x<-xx)sign=-1;
			
			if(sign>0){
						x-=deltat;
			}else{
						x+=deltat;
			}
			y=0.1*sin(x*omg);
			left_stepper_pulse_abs = atan(y/h)*6400.f/(2*3.1415926f);	
			right_stepper_pulse_abs = atan(x/h)*6400.f/(2*3.1415926f);	



#if 0
        if(left_stepper_pulse>0) //对电机来说 默认逆时针为正 
        {
            Emm_V5_Pos_Control(&stepper_motor_1, 1, motor_vel, motor_acc, left_stepper_pulse, 0, 0);
        }
        else if(left_stepper_pulse<0)

        {
            Emm_V5_Pos_Control(&stepper_motor_1, 0, motor_vel, motor_acc, -left_stepper_pulse, 0, 0);
        }
        if(right_stepper_pulse>0)
        {
            Emm_V5_Pos_Control(&stepper_motor_2, 1, motor_vel, motor_acc, right_stepper_pulse, 0, 0);
        }
        else if(right_stepper_pulse<0)

        {
            Emm_V5_Pos_Control(&stepper_motor_2, 0, motor_vel, motor_acc, -right_stepper_pulse, 0, 0);
        }
					left_stepper_pulse=0;
					right_stepper_pulse=0;
#else

				
				static int left_stepper_pulse_abs_last=0;
				static int right_stepper_pulse_abs_last=0;
				static float kk=0.23f;
				//motor_vel = abs(left_stepper_pulse_abs - left_stepper_pulse_abs_last)*kk;
        if(left_stepper_pulse_abs>0) //对电机来说 默认逆时针为正 
        {
            Emm_V5_Pos_Control(&stepper_motor_1, 1, motor_vel, motor_acc, left_stepper_pulse_abs, 1, 0);
        }
        else if(left_stepper_pulse_abs<0)

        {
            Emm_V5_Pos_Control(&stepper_motor_1, 0, motor_vel, motor_acc, -left_stepper_pulse_abs, 1, 0);
        }
				//motor_vel = abs(right_stepper_pulse_abs - right_stepper_pulse_abs_last)*kk;
        if(right_stepper_pulse_abs>0)
        {
            Emm_V5_Pos_Control(&stepper_motor_2, 1, motor_vel, motor_acc, right_stepper_pulse_abs, 1, 0);
        }
        else 

        {
            Emm_V5_Pos_Control(&stepper_motor_2, 0, motor_vel, motor_acc, -right_stepper_pulse_abs, 1, 0);
        }
				left_stepper_pulse_abs_last = left_stepper_pulse_abs;
				right_stepper_pulse_abs_last = right_stepper_pulse_abs;
			#endif	
        rt_thread_mdelay(1);//150有点震
    }
}

int emm_v5_init(void)
{

    /*互斥锁*/
    mutex_step = rt_mutex_create("mutex_emm", RT_IPC_FLAG_PRIO);
    if (mutex_step == RT_NULL)
        return -RT_ERROR;
    rt_thread_t drv_emm_v5_tb = RT_NULL;

    drv_emm_v5_tb = rt_thread_create("drv_emm_v5",
                                     drv_emm_v5_entry, RT_NULL,
                                     8192,
                                     5, 1);

    if (drv_emm_v5_tb != RT_NULL)
    {
        rt_thread_startup(drv_emm_v5_tb);
    }
    return 0;
}
//INIT_APP_EXPORT(emm_v5_init);