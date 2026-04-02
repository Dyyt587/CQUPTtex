#include "gripper.h"
#include "ulog.h"

//Gripper_Set_Angle(30);	//30度为启动时的起始位置
//Gripper_Set_Angle(70);	//60度-70度为张开时位置
//Gripper_Set_Angle(90);//90-92度刚好能夹住

#define PWM_DEV_NAME        "pwm2" /* PWM设 备 名 称 */
#define PWM_DEV_CHANNEL     1 << 0/* PWM通 道 */
 
struct rt_device_pwm *pwm_dev; /* PWM设 备 句 柄 */

//angle:初始角度
void Gripper_Init(float angle)
{
	  rt_uint32_t period=0, pulse=0;
    period = 20000000; /* 周 期 为50Hz,20ms， 单 位 为 纳 秒ns */
    //pulse = 10000000; /* PWM脉 冲 宽 度 值， 单 位 为 纳秒ns */
		pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
		if (pwm_dev == RT_NULL)
    {
        LOG_E("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
    }
		//rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
		Gripper_Set_Angle(angle);	//起始位置
		rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
}
//舵机设置角度angle:0-180
//
void Gripper_Set_Angle(float angle)
{
	rt_uint32_t pulse=(rt_uint32_t)(angle/180.0f*2000000)+500000;
	rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 20000000, pulse);
}
