/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <perf_counter.h>
#include <board.h>
#include <ulog.h>
#include <fal.h>
#include <flashdb.h>
#include "laser_gimbal.h"
#include "laser_gimbal_vision_tracker.h"
#include "hipnuc_dec.h"

// #include <lcd_st7789.h>
// #include "user_mb_app.h"
// #include "SM2924_Modbus.h"
// #include	"calculation.h"

/* defined the LED0 pin: PC13 */
#define LED0_PIN GET_PIN(C, 13)
#define LED1_PIN GET_PIN(D, 11)
#define LED2_PIN GET_PIN(D, 12)

#define BUZZER_PIN GET_PIN(D, 13)

// extern USHORT   usMRegHoldBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];

//	#define SAMPLE_UART_NAME       "uart2"
//	static rt_device_t serial;
// rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size) {
//	uint8_t data[size];
//		rt_device_read(dev,0,data,size);
//		rt_device_write(dev,0,data,size);
//    return RT_EOK;
//}

// static void thread1_entry(void *parameter)
//{
////    rt_uint32_t count = 0;

////    for (count = 0; count < 10 ; count++)
////    {

////        rt_kprintf("thread1 count: %d\n", count);
////        rt_thread_mdelay(500);
////    }
////    rt_kprintf("thread1 exit\n");

//	//rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
//	eMBMasterInit(MB_RTU,2,115200,MB_PAR_NONE);

//	eMBMasterEnable();

////	uint8_t str[10]="laiyang\n";
////				serial = rt_device_find(SAMPLE_UART_NAME);
////			rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
////	rt_device_set_rx_indicate(serial, serial_rx_ind);
//	for(;;)
//	{

//				//rt_device_write(serial, 0, str, (sizeof(str) - 1));
//			eMBMasterPoll();
//			//rt_kprintf("thread1_runing\n");
////			rt_pin_write(LED0_PIN, PIN_HIGH);
////			rt_thread_mdelay(100);

////			rt_pin_write(LED0_PIN, PIN_LOW);
//			rt_thread_mdelay(1);
//	}
//}

// rt_uint16_t error_count = 0;
// static void thread2_entry(void *parameter)
//{
//	 eMBMasterReqErrCode error_code = MB_MRE_NO_ERR;
//
//    USHORT data[2] = {0};
//	 rt_thread_mdelay(1000);	//延时等待总线启动
//	 uint16_t pos=0;
//	 uint16_t temp=0;
//	 float voltage=0;
//	 Motor_FTInit(&Motor_FT2,2);	//电机初始化
//	 Motor_FTSetPosOffSet(&Motor_FT2,3244);	//电机设置偏移量
//	 Motor_FTSetPolar(&Motor_FT2,-1);				//电机设置极性
//	 Motor_FTReadSynchronous(&Motor_FT2);
//
//	 Motor_FTInit(&Motor_FT3,3);
//	 Motor_FTSetPosOffSet(&Motor_FT3,2170);
//	 Motor_FTSetPolar(&Motor_FT3,1);				//电机设置极性
//	 Motor_FTReadSynchronous(&Motor_FT3);
//
//	 Motor_FTInit(&Motor_FT4,4);
//	 Motor_FTSetPosOffSet(&Motor_FT4,2170);
//	 Motor_FTSetPolar(&Motor_FT4,1);				//电机设置极性
//	 Motor_FTReadSynchronous(&Motor_FT4);
//	 double angle1,angle2;
//	 //Motor_FT2.
//	for(;;)
//	{
//		data[0]=0x0002;
//		//data[1]=0x2222;
//		error_count++;

////		Motor_FT1.control.pos=1000;
//
//		//rt_kprintf("err_code:%d\n",error_code);
//		//error_code=eMBMasterReqWriteMultipleHoldingRegister(0x01,0x000B,1,data,100);
//		//error_code=eMBMasterReqReadHoldingRegister(0x01,0x000b,1,100);
//		//error_code=eMBMasterReqWriteHoldingRegister(0x01,0x0080,100,50);
//		//error_code=eMBMasterReqReadHoldingRegister(0x01,0x0080,6,100);
//		//error_code=eMBMasterReqReadHoldingRegister(0x04,0x0100,8,100);
//
////		Motor_FTSetPos(&Motor_FT4,-1000);
////		Motor_FTWriteSynchronous(&Motor_FT4);
////
////		Motor_FTSetPos(&Motor_FT3,1000);
////		Motor_FTWriteSynchronous(&Motor_FT3);
////
//		//Motor_FTSetPos(&Motor_FT2,-800);
////		Motor_FTSetAngle(&Motor_FT2,3.14/2);
////		Motor_FTWriteSynchronous(&Motor_FT2);
////
////		Motor_FTSetAngle(&Motor_FT3,0);
////		Motor_FTWriteSynchronous(&Motor_FT3);
//
//		uint8_t retval=inverse_kinematics(0.2828,0,&angle1,&angle2);
//		if(retval==0)
//		{
//			LOG_D("Cannot find inserve");
//		}
//		else{
//			Motor_FTSetAngle(&Motor_FT2,angle1);
//			Motor_FTWriteSynchronous(&Motor_FT2);
//
//			Motor_FTSetAngle(&Motor_FT3,angle2);
//			Motor_FTWriteSynchronous(&Motor_FT3);
//			LOG_D("%.2f,%.2f",angle1,angle2);
//		}
//
////		Motor_FTSetAngle(&Motor_FT2,angle1_1);
////		Motor_FTWriteSynchronous(&Motor_FT2);
////
////		Motor_FTSetAngle(&Motor_FT3,angle2_1);
////		Motor_FTWriteSynchronous(&Motor_FT3);
//
//		if (error_code != MB_MRE_NO_ERR)
//		{
//       //error_count++;
//			LOG_D("err_code:%d\n",error_code);
//
//    }
//
//		//pos=usMRegHoldBuf[0][0];
//		//voltage=0.1*usMRegHoldBuf[0][1];
//		//temp=usMRegHoldBuf[0][2];
//		//LOG_D("%d,%d,%d,%d",usMRegHoldBuf[0][0x100],usMRegHoldBuf[0][0x101],usMRegHoldBuf[0][0x82],usMRegHoldBuf[0][0x83]);
//		error_code=Motor_FTReadValid(&Motor_FT2);
//		error_code=Motor_FTReadValid(&Motor_FT3);
//		error_code=Motor_FTReadValid(&Motor_FT4);
//		if (error_code != MB_MRE_NO_ERR)
//		{
//       //error_count++;
//			LOG_D("err_code:%d\n",error_code);
//
//    }
//		//LOG_D("%d,%d,%d,%d",Motor_FT2.statues.pos,Motor_FT2.statues.speed,Motor_FT3.statues.pos,Motor_FT4.statues.pos);
//		//LOG_D("%.2f,%.2f",angle1,angle2);
//		rt_thread_mdelay(1);
//	}
//}

// #define SCK_PIN    GET_PIN(C, 1)
// #define SCL_PIN    GET_PIN(C, 3)
// #define SCL1_PIN    GET_PIN(C, 2)

int main(void)
{
  /* set LED0 pin mode to output */
  rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
  rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
  rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
  rt_pin_mode(BUZZER_PIN, PIN_MODE_OUTPUT);

  // extern int mb_master_sample(int argc, char **argv);

  bool date[16];

		extern rt_err_t laser_gimbal_system_init(void);
	rt_err_t result;
		TRY:
	  result = laser_gimbal_system_init();
    if (result == RT_EOK)
    {
        LOG_D("Laser gimbal system initialized successfully\n");
    }
    else
    {
        LOG_D("Failed to initialize laser gimbal system: %d try anain \n", result);
			goto TRY;
    }
		extern void vision_init(int argc, char **argv);
	        extern vision_tracker_t g_vision_tracker;

		vision_init(1,0);
	g_vision_tracker.mode = VISION_TRACK_MODE_CONTINUOUS;
  while (1)
  {

    // LOG_HEX("example",16,date,16);
    // rt_thread_mdelay(5);
  rt_pin_write(BUZZER_PIN, PIN_HIGH);

    rt_pin_write(LED0_PIN, PIN_HIGH);
    rt_thread_mdelay(100);

		    rt_err_t result = vision_tracker_enable(&g_vision_tracker, 1);

    rt_pin_write(LED0_PIN, PIN_LOW);
    rt_thread_mdelay(100);

    rt_pin_write(LED1_PIN, PIN_LOW);
    rt_thread_mdelay(100);

    rt_pin_write(LED1_PIN, PIN_HIGH);
    rt_thread_mdelay(100);

    rt_pin_write(LED2_PIN, PIN_LOW);
    rt_thread_mdelay(100);

    rt_pin_write(LED2_PIN, PIN_HIGH);
    rt_thread_mdelay(100);

    //			  rt_pin_write(BUZZER_PIN, PIN_HIGH);
    //        rt_thread_mdelay(500);
    //
    //				rt_pin_write(BUZZER_PIN, PIN_LOW);
    //        rt_thread_mdelay(500);
  }
}



//INIT_ENV_EXPORT(laser_init);








static uint32_t boot_count = 0;
struct fdb_kvdb _global_kvdb = {0};
struct fdb_tsdb _global_tsdb = {0};
extern void kvdb_basic_sample(fdb_kvdb_t kvdb);
static struct fdb_default_kv_node default_kv_table[] = {{"boot_count", &boot_count, sizeof(boot_count)}}; /* int type KV */

int flashdb(void)
{
  fdb_err_t result;
  struct fdb_default_kv default_kv;
  default_kv.kvs = default_kv_table;
  default_kv.num = sizeof(default_kv_table) / sizeof(default_kv_table[0]);
    fal_init();

  result = fdb_kvdb_init(&_global_kvdb, "env", "app", &default_kv, NULL);
  // init函数的第三个参数，是用于存储数据的fal分区，大家根据自己的情况选择
  if (result != FDB_NO_ERR)
  {
    return -1;
  }
  else
  {
    kvdb_basic_sample(&_global_kvdb);
  }
  return RT_EOK;
}
//INIT_ENV_EXPORT(flashdb);

void buzzer_on(int argc, char **argv)
{
  //		hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
  //    LOG_D("parse ok, frame len:%d\r\n", hipnuc_raw.len);
  //    LOG_D("%s\r\n", log_buf);
  // LOG_D("Roll:%.5f,Pitch:%.5f,Yaw:%.5f",hipnuc_raw.hi91.roll,hipnuc_raw.hi91.pitch,hipnuc_raw.hi91.yaw);
  rt_pin_write(BUZZER_PIN, PIN_HIGH);
}
MSH_CMD_EXPORT(buzzer_on, BUZZER ON);

 void buzzer_off(int argc, char **argv)
{
  //		hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
  //    LOG_D("parse ok, frame len:%d\r\n", hipnuc_raw.len);
  //    LOG_D("%s\r\n", log_buf);
  // LOG_D("Roll:%.5f,Pitch:%.5f,Yaw:%.5f",hipnuc_raw.hi91.roll,hipnuc_raw.hi91.pitch,hipnuc_raw.hi91.yaw);
  rt_pin_write(BUZZER_PIN, PIN_LOW);
}
MSH_CMD_EXPORT(buzzer_off, BUZZER OFF);

/* ======================== 增强型陀螺仪补偿测试命令 ======================== */

/**
 * @brief 显示当前的增强型补偿参数和状态
 */
static void gyro_comp_status(int argc, char **argv)
{
    extern vision_tracker_t g_vision_tracker;
    extern hipnuc_raw_t hipnuc_raw;
    
    rt_kprintf("=== Enhanced Gyro Compensation Status ===\n");
    
    /* 显示当前IMU数据 */
    rt_kprintf("Current IMU Data:\n");
    rt_kprintf("  Angles: Yaw=%.2f° Pitch=%.2f° Roll=%.2f°\n", 
               hipnuc_raw.hi91.yaw, hipnuc_raw.hi91.pitch, hipnuc_raw.hi91.roll);
    rt_kprintf("  Angular Velocity: X=%.2f°/s Y=%.2f°/s Z=%.2f°/s\n",
               hipnuc_raw.hi91.gyr[0], hipnuc_raw.hi91.gyr[1], hipnuc_raw.hi91.gyr[2]);
    
    /* 显示补偿设置 */
    rt_kprintf("Compensation Settings:\n");
    rt_kprintf("  Enabled: %s\n", g_vision_tracker.gyro_compensation_enabled ? "YES" : "NO");
    rt_kprintf("  Yaw Gain: %.3f\n", g_vision_tracker.gyro_compensation_gain_yaw);
    rt_kprintf("  Pitch Gain: %.3f\n", g_vision_tracker.gyro_compensation_gain_pitch);
    rt_kprintf("  Motion Threshold: %.1f°/s\n", g_vision_tracker.gyro_motion_threshold);
    
    rt_kprintf("Enhanced Features:\n");
    rt_kprintf("  - Angle + Angular Velocity + Acceleration Fusion\n");
    rt_kprintf("  - Dynamic Weight Adjustment\n");
    rt_kprintf("  - Anti-Windup Protection\n");
    rt_kprintf("  - Acceleration Threshold: 50°/s²\n");
    rt_kprintf("  - Max Compensation: ±10°\n");
}
MSH_CMD_EXPORT(gyro_comp_status, Show enhanced gyro compensation status);

/**
 * @brief 设置补偿权重 - 用于实时调试
 * 使用方法: comp_weight <angle_weight> <vel_weight> <accel_weight>
 */
static void comp_weight(int argc, char **argv)
{
    if (argc != 4) {
        rt_kprintf("Usage: comp_weight <angle_weight> <vel_weight> <accel_weight>\n");
        rt_kprintf("Example: comp_weight 0.2 0.6 0.2\n");
        rt_kprintf("Note: Weights should sum to 1.0\n");
        return;
    }
    
    float angle_w = atof(argv[1]);
    float vel_w = atof(argv[2]); 
    float accel_w = atof(argv[3]);
    
    float total = angle_w + vel_w + accel_w;
    if (total < 0.9f || total > 1.1f) {
        rt_kprintf("Warning: Weights sum to %.2f (should be close to 1.0)\n", total);
    }
    
    rt_kprintf("Setting compensation weights:\n");
    rt_kprintf("  Angle: %.2f  Velocity: %.2f  Acceleration: %.2f\n", angle_w, vel_w, accel_w);
    rt_kprintf("Note: These will be used in next compensation cycle\n");
    rt_kprintf("Use 'gyro_comp_status' to monitor real-time effects\n");
}
MSH_CMD_EXPORT(comp_weight, Set compensation weights (angle vel accel));

/**
 * @brief 模拟突然加速测试
 */
static void test_accel_compensation(int argc, char **argv)
{
    extern vision_tracker_t g_vision_tracker;
    extern hipnuc_raw_t hipnuc_raw;
    
    rt_kprintf("=== Acceleration Compensation Test ===\n");
    rt_kprintf("Monitoring for sudden acceleration events...\n");
    rt_kprintf("Move the platform suddenly to test compensation\n");
    rt_kprintf("Press Ctrl+C to stop monitoring\n\n");
    
    static float last_gyro[3] = {0};
    static uint32_t last_time = 0;
    const float accel_threshold = 50.0f; // 同代码中的阈值
    
    for (int i = 0; i < 100; i++) { // 监控10秒
        uint32_t current_time = rt_tick_get_millisecond();
        float dt = (current_time - last_time) / 1000.0f;
        
        if (dt > 0.001f && dt < 0.1f) { // 有效时间间隔
            /* 计算角加速度 */
            float accel_x = (hipnuc_raw.hi91.gyr[0] - last_gyro[0]) / dt;
            float accel_y = (hipnuc_raw.hi91.gyr[1] - last_gyro[1]) / dt;
            float accel_z = (hipnuc_raw.hi91.gyr[2] - last_gyro[2]) / dt;
            
            /* 检测是否超过阈值 */
            if (fabsf(accel_y) > accel_threshold || fabsf(accel_z) > accel_threshold) {
                rt_kprintf("*** SUDDEN ACCELERATION DETECTED! ***\n");
                rt_kprintf("  Pitch Accel: %.1f°/s² (threshold: %.1f)\n", accel_y, accel_threshold);
                rt_kprintf("  Yaw Accel: %.1f°/s² (threshold: %.1f)\n", accel_z, accel_threshold);
                rt_kprintf("  Enhanced compensation should be activated\n\n");
            } else {
                rt_kprintf("Normal motion - Accel: P=%.1f Y=%.1f°/s²\r", accel_y, accel_z);
            }
            
            last_gyro[0] = hipnuc_raw.hi91.gyr[0];
            last_gyro[1] = hipnuc_raw.hi91.gyr[1];
            last_gyro[2] = hipnuc_raw.hi91.gyr[2];
        }
        
        last_time = current_time;
        rt_thread_mdelay(100); // 10Hz监控频率
    }
    
    rt_kprintf("\nAcceleration test completed.\n");
}
MSH_CMD_EXPORT(test_accel_compensation, Test sudden acceleration compensation);

/**
 * @brief 补偿效果对比测试
 */
static void comp_compare(int argc, char **argv)
{
    extern vision_tracker_t g_vision_tracker;
    
    rt_kprintf("=== Compensation Effect Comparison ===\n");
    rt_kprintf("This test will toggle compensation on/off to show differences\n");
    
    bool original_state = g_vision_tracker.gyro_compensation_enabled;
    
    /* 关闭补偿测试5秒 */
    rt_kprintf("\n1. Disabling compensation for 5 seconds...\n");
    g_vision_tracker.gyro_compensation_enabled = false;
    rt_kprintf("   Move the platform and observe gimbal stability\n");
    for (int i = 5; i > 0; i--) {
        rt_kprintf("   Time remaining: %d seconds\r", i);
        rt_thread_mdelay(1000);
    }
    
    /* 开启补偿测试5秒 */
    rt_kprintf("\n\n2. Enabling enhanced compensation for 5 seconds...\n");
    g_vision_tracker.gyro_compensation_enabled = true;
    rt_kprintf("   Move the platform and compare stability\n");
    for (int i = 5; i > 0; i--) {
        rt_kprintf("   Time remaining: %d seconds\r", i);
        rt_thread_mdelay(1000);
    }
    
    /* 恢复原始状态 */
    g_vision_tracker.gyro_compensation_enabled = original_state;
    rt_kprintf("\n\nComparison test completed. Original state restored.\n");
    rt_kprintf("Enhanced compensation should provide better stability during sudden movements.\n");
}
MSH_CMD_EXPORT(comp_compare, Compare compensation on/off effects);
