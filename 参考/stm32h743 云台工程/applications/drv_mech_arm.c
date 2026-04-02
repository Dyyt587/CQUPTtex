#include "drv_mech_arm.h"
#include "SM2924_Modbus.h"
#include "calculation.h"
#include "gripper.h"
#include "key.h"
#include "meArm.h"
#include "user_mb_app.h"
#include <ulog.h>
#include "drv_gray_sensor.h"

static rt_thread_t thread_modbus_loop = RT_NULL; // modbus总线线程
static rt_thread_t thread_mech_arm = RT_NULL;    // 机械臂操作线程

//uint16_t arm_run_flag = 0;    // arm运行开始标志位
//uint16_t arm_finish_flag = 1; // arm运行结束标志位

static void modbus_loop_entry(void *parameter) {

  eMBMasterInit(MB_RTU, 2, 115200, MB_PAR_NONE);

  eMBMasterEnable();

  //	uint8_t str[10]="laiyang\n";
  //				serial = rt_device_find(SAMPLE_UART_NAME);
  //			rt_device_open(serial, RT_DEVICE_OFLAG_RDWR |
  //RT_DEVICE_FLAG_INT_RX); 	rt_device_set_rx_indicate(serial, serial_rx_ind);
  for (;;) {

    // rt_device_write(serial, 0, str, (sizeof(str) - 1));
    eMBMasterPoll();
    // rt_kprintf("thread1_runing\n");
    //			rt_pin_write(LED0_PIN, PIN_HIGH);
    //			rt_thread_mdelay(100);

    //			rt_pin_write(LED0_PIN, PIN_LOW);
    rt_thread_mdelay(1);
  }
}

rt_uint16_t error_count = 0;

static void mech_arm_entry(void *parameter) {
  eMBMasterReqErrCode error_code = MB_MRE_NO_ERR;

  USHORT data[2] = {0};
  rt_thread_mdelay(2000); // 延时等待总线启动

  Motor_FTInit(&Motor_FT_Left, 1);
  Motor_FTSetPosOffSet(&Motor_FT_Left, 0);
  Motor_FTSetPolar(&Motor_FT_Left, 1); // 电机设置极性
  Motor_FTReadSynchronous(&Motor_FT_Left);

  Motor_FTInit(&Motor_FT_Right, 2);
  Motor_FTSetPosOffSet(&Motor_FT_Right,0);
  Motor_FTSetPolar(&Motor_FT_Right, 1); // 电机设置极性
  Motor_FTReadSynchronous(&Motor_FT_Right);

  meArm_Init();
	rt_thread_mdelay(200);
  for (;;) {

    if (arm_run_flag == 0) // 机械臂不操作
    {
      error_code = Motor_FTReadValid(&Motor_FT_Left);
      error_code = Motor_FTReadValid(&Motor_FT_Right);

      if (error_code != MB_MRE_NO_ERR) {
        //error_count++;
        // if(error_count<10) LOG_D("err_code:%d\n",error_code);
        //LOG_D("err_code:%d\n", error_code);
      }
//       LOG_D("motor_angle:%.2f,%.2f", Motor_FT_Left.statues.angle,
//             Motor_FT_Right.statues.angle);
			
//			 LOG_D("motor_pos:%d,%d", Motor_FT_Left.statues.pos,
//             Motor_FT_Right.statues.pos);
       //LOG_D("arm_finish_flag:%d", arm_finish_flag);

      rt_thread_mdelay(100);
    } 
		Motor_FTSetPos(&Motor_FT_Left,0);
		Motor_FTWriteSynchronous(&Motor_FT_Left);
		Motor_FTSetPos(&Motor_FT_Right,3000);
		Motor_FTWriteSynchronous(&Motor_FT_Right);
		
//		else if (arm_run_flag == 1 && arm_finish_flag == 0) // 夹取操作
//    {
//      meArm_Gripper_Get();
//      arm_run_flag = 0;
//      arm_finish_flag = 1;
//    } 
//		else if (arm_run_flag == 2 && arm_finish_flag == 0) // 放置到1分区
//    {
//      meArm_Gripper_Release_1();
//      arm_run_flag = 0;
//      arm_finish_flag = 1;
//    } 
//		else if (arm_run_flag == 3 && arm_finish_flag == 0) // 放置到2分区,先进行位置矫正
//    {
//			chassis_gray_flag=1;
//			//uint8_t gray_count=0;
//			while(!(chassis_gray_flag==0&&chassis_gray_finish_flag==1))
//			{
////				gray_count++;
////				if(gray_count>=10)
////				{
////					break;
////				}
//				rt_thread_mdelay(100);
//			}
//			chassis_gray_finish_flag=0;
//      meArm_Gripper_Release_2();
//      arm_run_flag = 0;
//      arm_finish_flag = 1;

//    } 
//		else if (arm_run_flag == 4 &&
//               arm_finish_flag == 0) // 放置到3分区的第一种模式
//    {
//      meArm_Gripper_Release_3_1();
//      arm_run_flag = 0;
//      arm_finish_flag = 1;
//    } 
//		else if (arm_run_flag == 5 &&
//               arm_finish_flag == 0) // 放置到3分区的第二种模式
//    {
//      meArm_Gripper_Release_3_2();
//      arm_run_flag = 0;
//      arm_finish_flag = 1;
//    }

//		if(KEY_FLAG==0)
//		{
//			rt_thread_mdelay(20);
//		}
//		else if(KEY_FLAG==1)
//		{
//			meArm_Gripper_Get();
//			KEY_FLAG=0;
//		}
//		else if(KEY_FLAG==2)
//		{
//			meArm_Gripper_Release_3_1();
//			KEY_FLAG=0;
//		}
//		else if(KEY_FLAG==3)
//		{
//			meArm_Gripper_Release_3_2();
//			KEY_FLAG=0;
//		}
  }
}
int arm_init(void) {

  thread_modbus_loop =
      rt_thread_create("modbus_loop", modbus_loop_entry, RT_NULL, 1024, 10, 1);

  if (thread_modbus_loop != RT_NULL) {
    rt_thread_startup(thread_modbus_loop);
  }

  thread_mech_arm =
      rt_thread_create("mech_arm", mech_arm_entry, RT_NULL, 1024, 9, 1);

  if (thread_mech_arm != RT_NULL) {
    rt_thread_startup(thread_mech_arm);
  }
  return 0;
}
INIT_APP_EXPORT(arm_init);