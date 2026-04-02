#include "pwm.h"
#include "ulog.h"



#define PWM_DEV_2        			"pwm2" 	/* PWM设 备 名 称 */
#define PWM_DEV_3        			"pwm3" 	/* PWM设 备 名 称 */
#define PWM_DEV_5        			"pwm5" 	/* PWM设 备 名 称 */

#define PWM_DEV_CHANNEL_1     1 	/* PWM通 道 */
#define PWM_DEV_CHANNEL_2     2	/* PWM通 道 */
#define PWM_DEV_CHANNEL_3     3	/* PWM通 道 */
#define PWM_DEV_CHANNEL_4     4	/* PWM通 道 */


 
static struct rt_device_pwm *pwm_dev_2; /* PWM设 备 句 柄 */
static struct rt_device_pwm *pwm_dev_3; /* PWM设 备 句 柄 */
static struct rt_device_pwm *pwm_dev_5; /* PWM设 备 句 柄 */
static rt_thread_t thread_pwm=RT_NULL;	//pwm线程 


//PWM线程入口函数
static void thread_pwm_entry(void *parameter)
{
    rt_uint32_t period, pulse, dir;

    period = 500000;    /* 周期为0.5ms，单位为纳秒ns */
    pulse = 0;          /* PWM脉冲宽度值，单位为纳秒ns */

    /* 查找设备 */
    pwm_dev_2 = (struct rt_device_pwm *)rt_device_find(PWM_DEV_2);
    if (pwm_dev_2 == RT_NULL)
    {
        LOG_D("pwm sample run failed! can't find %s device!\n", PWM_DEV_2);
    }
		pwm_dev_3 = (struct rt_device_pwm *)rt_device_find(PWM_DEV_3);
    if (pwm_dev_3 == RT_NULL)
    {
        LOG_D("pwm sample run failed! can't find %s device!\n", PWM_DEV_3);
    }
		pwm_dev_5 = (struct rt_device_pwm *)rt_device_find(PWM_DEV_5);
    if (pwm_dev_5 == RT_NULL)
    {
        LOG_D("pwm sample run failed! can't find %s device!\n", PWM_DEV_5);
    }

		//定时器2
		//初始化设置
    rt_pwm_set(pwm_dev_2, PWM_DEV_CHANNEL_1, period, pulse);
		rt_pwm_set(pwm_dev_2, PWM_DEV_CHANNEL_3, period, pulse);
		rt_pwm_set(pwm_dev_2, PWM_DEV_CHANNEL_4, period, pulse);
		//使能
    rt_pwm_enable(pwm_dev_2, PWM_DEV_CHANNEL_1);
		rt_pwm_enable(pwm_dev_2, PWM_DEV_CHANNEL_3);
		rt_pwm_enable(pwm_dev_2, PWM_DEV_CHANNEL_4);
		
		//定时器3
		//初始化设置
		rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_1, period, pulse);
		rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_3, period, pulse);
		rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_3, period, pulse);
		rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_4, period, pulse);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev_3, PWM_DEV_CHANNEL_1);
		rt_pwm_enable(pwm_dev_3, PWM_DEV_CHANNEL_2);
		rt_pwm_enable(pwm_dev_3, PWM_DEV_CHANNEL_3);
		rt_pwm_enable(pwm_dev_3, PWM_DEV_CHANNEL_4);
		
		
		//定时器5
		//初始化设置
		rt_pwm_set(pwm_dev_5, PWM_DEV_CHANNEL_1, period, pulse);
		rt_pwm_set(pwm_dev_5, PWM_DEV_CHANNEL_2, period, pulse);
		rt_pwm_set(pwm_dev_5, PWM_DEV_CHANNEL_3, period, pulse);

    /* 使能设备 */
    rt_pwm_enable(pwm_dev_5, PWM_DEV_CHANNEL_1);
		rt_pwm_enable(pwm_dev_5, PWM_DEV_CHANNEL_2);
		rt_pwm_enable(pwm_dev_5, PWM_DEV_CHANNEL_3);


    while (1)
    {
        /* 设置PWM周期和脉冲宽度 */
        rt_pwm_set(pwm_dev_2, PWM_DEV_CHANNEL_1, period, period*0.2);
				rt_pwm_set(pwm_dev_2, PWM_DEV_CHANNEL_3, period, period*0.5);
				rt_pwm_set(pwm_dev_2, PWM_DEV_CHANNEL_4, period, period*0.8);
			
				rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_1, period, period*0.2);
				rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_2, period, period*0.4);
				rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_3, period, period*0.6);
				rt_pwm_set(pwm_dev_3, PWM_DEV_CHANNEL_4, period, period*0.8);
			
			  rt_pwm_set(pwm_dev_5, PWM_DEV_CHANNEL_1, period, period*0.2);
				rt_pwm_set(pwm_dev_5, PWM_DEV_CHANNEL_2, period, period*0.5);
				rt_pwm_set(pwm_dev_5, PWM_DEV_CHANNEL_3, period, period*0.8);
			
				rt_thread_mdelay(50);
    }
}

int pwm_init(void) {

  thread_pwm=
      rt_thread_create("thread_pwm", thread_pwm_entry, RT_NULL, 1024, 9, 1);

  if (thread_pwm != RT_NULL) {
    rt_thread_startup(thread_pwm);
  }
  return 0;
}
//INIT_APP_EXPORT(pwm_init);
