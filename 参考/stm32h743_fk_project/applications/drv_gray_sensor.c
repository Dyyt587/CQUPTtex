#include "drv_gray_sensor.h"
#include <perf_counter.h>
#include <board.h>
#include "ulog.h"
#include <stdlib.h>
#include "apid.h"

#include "meArm.h"

#define PID_POSITION APID_POSITION 
rt_thread_t drv_gray_sensor=RT_NULL;

uint8_t chassis_gray_flag=0;				//外部控制的校准标志位
uint8_t chassis_gray_finish_flag=0;	//反馈控制结束标志位

#define gray_abs(x) ((x>=0)?(x):(-x))
//定位矫正的PID
apid_t pid_gary_x;
apid_t pid_gary_y;
apid_t pid_gary_z;

const float weight_matrix[4][8]=
{
	{0.4,0.3,0.2,0.1,-0.1,-0.2,-0.3,-0.4},
	{0.4,0.3,0.2,0.1,-0.1,-0.2,-0.3,-0.4},
	{0.4,0.3,0.2,0.1,-0.1,-0.2,-0.3,-0.4},
	{0.4,0.3,0.2,0.1,-0.1,-0.2,-0.3,-0.4},
};


//#define gray_half_width 0.005f	//灰度传感器的半宽（单位：m）
//#define gray_half_x 		0.006f	//灰度传感器支架的x轴半宽
//#define gray_half_y			0.005f	//灰度传感器支架的y轴半宽
#define gray_half_width 4.5f	//灰度传感器的半宽（单位：cm）
#define gray_half_x 		7.0f	//灰度传感器支架的x轴半宽
#define gray_half_y			6.0f	//灰度传感器支架的y轴半宽
//灰度摆放位置
//			---------1
//			|       |
//			|				|
//		4	|				| 2
//			|				|
//			---------
//					3
//板子实际
//PI0/PI2/PI4/PI6  --SCK  --OUTPUT
//PI1/PI3/PI5/PI7  --SCL	--INPUT

#define SCK_Front GET_PIN(I,0)
#define SCL_Front GET_PIN(I,1)

#define SCK_Right GET_PIN(I,2)
#define SCL_Right GET_PIN(I,3)

#define SCK_Rear GET_PIN(I,4)
#define SCL_Rear GET_PIN(I,5)

#define SCK_Left GET_PIN(I,6)
#define SCL_Left GET_PIN(I,7)


//计算x,y,w位置偏移
void count_offset(bool gray_data[32],float *x,float *y,float *w)
{
	float weight_value[4]={0,0,0,0};
	float w1=0,w2=0;
	for(uint8_t i=0;i<4;i++)
	{
		for(uint8_t j=0;j<8;j++)
		{
			weight_value[i]+=weight_matrix[i][j]*gray_data[i*8+j];
		}
	}
	//量化
	for(uint8_t i=0;i<4;i++)
	{
		weight_value[i]=weight_value[i]*gray_half_width;
	}
	float k1=0,k2=0,b1=0,b2=0;
	//计算前后两个灰度形成的直线
	if((weight_value[0]-weight_value[2])>=-0.001f&&(weight_value[0]-weight_value[2])<=0.001f)
	{
		k1=0;
		b1=(weight_value[0]+weight_value[2])/2;
	}
	else{
			k1=(2*gray_half_y)/(weight_value[0]-weight_value[2]);
			b1=gray_half_y-k1*weight_value[0];
	}
	//计算左后两个灰度形成的直线
	if((weight_value[1]-weight_value[3])>=-0.001f&&(weight_value[1]-weight_value[3])<=0.001f)
	{
			k2=0;
			b2=(weight_value[1]+weight_value[3])/2;
	}
	else{
			k2=(2*gray_half_x)/(weight_value[1]-weight_value[3]);
			b2=gray_half_x-k2*weight_value[1];
	}
	
	*x=b1;
	*y=b2;
	*w=(k1+k2)/2;
	//*x=weight_value[0];
	//*y=weight_value[1];
	//*w=0;
	//LOG_D("x:%.2f,y:%.2f,w:%.2f",*x,*y,*w);

		
	//LOG_D("%.2f,%.2f,%.2f,%.2f",\
	weight_value[0],weight_value[1],weight_value[2],weight_value[3]);
}

static void gray_sensor_entry(void *parameter)
{
	bool gray_sensor_date[32]={0};
	
//  rt_pin_mode(SCK_PIN, PIN_MODE_OUTPUT);
//  rt_pin_mode(SCL1_PIN, PIN_MODE_INPUT);
//  rt_pin_mode(SCL2_PIN, PIN_MODE_INPUT);
//	rt_pin_mode(SCL3_PIN, PIN_MODE_INPUT);
//	rt_pin_mode(SCL4_PIN, PIN_MODE_INPUT);
//	rt_pin_write(SCK_PIN, PIN_HIGH);
	
	rt_pin_mode(SCK_Front, PIN_MODE_OUTPUT);
	rt_pin_mode(SCK_Right, PIN_MODE_OUTPUT);
	rt_pin_mode(SCK_Rear, PIN_MODE_OUTPUT);
	rt_pin_mode(SCK_Left, PIN_MODE_OUTPUT);
	
  rt_pin_mode(SCL_Front, PIN_MODE_INPUT);
  rt_pin_mode(SCL_Right, PIN_MODE_INPUT);
	rt_pin_mode(SCL_Rear, PIN_MODE_INPUT);
	rt_pin_mode(SCL_Left, PIN_MODE_INPUT);
	
	rt_pin_write(SCK_Front, PIN_HIGH);
	rt_pin_write(SCK_Rear, PIN_HIGH);
	rt_pin_write(SCK_Right, PIN_HIGH);
	rt_pin_write(SCK_Left, PIN_HIGH);
  rt_thread_mdelay(5);

	float x,y,z;
	//PID初始化
//	APID_Init(&pid_gary_x,PID_POSITION,0.01,0,0);
//	APID_Init(&pid_gary_y,PID_POSITION,0.01,0,0);
//	APID_Init(&pid_gary_z,PID_POSITION,0.01,0,0);
//	APID_Enable(&pid_gary_x); // 默认已经开启
//	APID_Enable(&pid_gary_y); // 默认已经开启
//	APID_Enable(&pid_gary_z); // 默认已经开启
//	
//	APID_Set_Target(&pid_gary_x, 0);
//	APID_Set_Target(&pid_gary_y, 0);
//	APID_Set_Target(&pid_gary_z, 0);
//	
//	APID_Set_Out_Limit(&pid_gary_x,0.1);
//	APID_Set_Out_Limit(&pid_gary_y,0.1);
//	APID_Set_Out_Limit(&pid_gary_z,0.1);

	float max_error[4]={0,0,0,0};
	int8_t max_index=0,min_index=0;
	uint8_t change_flag[4];	//修正flag,0时，不能进行位置修正，1：可以进行位置修改
	rt_thread_mdelay(2000);
	while(1)
	{
		//rt_enter_critical();			
			
		for(int i=0;i<8;i++)
		{
			rt_pin_write(SCK_Front, PIN_LOW);
			rt_pin_write(SCK_Rear, PIN_LOW);
			rt_pin_write(SCK_Right, PIN_LOW);
			rt_pin_write(SCK_Left, PIN_LOW);
			
			gray_sensor_date[i]=rt_pin_read(SCL_Front);
			gray_sensor_date[i+8]=rt_pin_read(SCL_Right);
			gray_sensor_date[i+16]=rt_pin_read(SCL_Rear);
			gray_sensor_date[i+24]=rt_pin_read(SCL_Left);
			
//			gray_sensor_date[i]=!gray_sensor_date[i];
//			gray_sensor_date[i+8]=!gray_sensor_date[i+8];
//			gray_sensor_date[i+16]=!gray_sensor_date[i+16];
//			gray_sensor_date[i+24]=!gray_sensor_date[i+24];
			
			rt_pin_write(SCK_Front, PIN_HIGH);
			rt_pin_write(SCK_Rear, PIN_HIGH);
			rt_pin_write(SCK_Right, PIN_HIGH);
			rt_pin_write(SCK_Left, PIN_HIGH);
			delay_us(5);
		}
		//rt_exit_critical();//退出调度临界区
		//count_offset(gray_sensor_date,&x,&y,&z);
		for(uint8_t i=0;i<4;i++)
		{
			
			max_error[i]=0;
			min_index=-1;	//最小的索引，从左向右
			max_index=-1;	//最大的索引，从右向左
			change_flag[i]=0;	//清零change_flag
			uint8_t num_total=0;
			for(uint8_t j=0;j<8;j++)
			{
				num_total+=gray_sensor_date[i*8+j];
			}
			if(num_total==0)	//灰度值全为0的情况，说明该方向偏移太大
			{
				
			}
			else if(num_total==8)	//灰度值全为8的情况，说明需要往该方向大偏移
			{
				
			}
			else//灰度值有黑与白，可以矫正
			{
					change_flag[i]=1;	//该灰度可以进行位置矫正
					if(gray_sensor_date[i*8+0]==1)
					{
						min_index=0;
					}
					if(gray_sensor_date[i*8+7]==1)
					{
						max_index=7;
					}
					for(uint8_t j=1;j<=6;j++)
					{
						if(min_index==-1&&gray_sensor_date[i*8+j-1]==0&&gray_sensor_date[i*8+j]==1)
						{
							min_index=j;
						}
						if(max_index==-1&&gray_sensor_date[i*8+j]==1&&gray_sensor_date[i*8+j+1]==0)
						{
							max_index=j;
						}
					}
					//计算最大的偏移值
					if(gray_abs(max_index-3.5)>=gray_abs(min_index-3.5))
					{
						max_error[i]=max_index-3.5;
					}
					else
					{
						max_error[i]=min_index-3.5;
					}
			}
		}
			//对应方向值应该反向
		max_error[2]*=-1;
		max_error[3]*=-1;
		
		if(chassis_gray_flag==1&&(change_flag[0]==1||change_flag[2]==1))	//进行位置矫正
		{
			if(change_flag[0]==1)	//前面可以进行矫正，x轴方向
			{
				car_run(0.01*(max_error[0]-0.5),0,0);
				rt_thread_mdelay(500);
			}
			else	//通过后面矫正
			{
				car_run(0.01*(max_error[2]-(-0.5)),0,0);
				rt_thread_mdelay(500);
			}	
		}
		if(chassis_gray_flag==1&&(change_flag[1]==1||change_flag[3]==1))	//进行位置矫正
		{
			if(change_flag[1]==1)	//右边可以进行矫正，y轴方向
			{
				car_run(0,-0.01*(max_error[1]-0.5),0);
				rt_thread_mdelay(500);
			}
			else	//通过左边矫正
			{
				car_run(0,-0.01*(max_error[3]-(-0.5)),0);
				rt_thread_mdelay(500);
			}
			chassis_gray_flag=0;
			chassis_gray_finish_flag=1;
		}
		//LOG_D("ERROR:%.2f,%.2f,%.2f,%.2f",max_error[0],max_error[1],max_error[2],max_error[3]);
		//LOG_D("CHAN:%d,%d,%d,%d",change_flag[0],change_flag[1],change_flag[2],change_flag[3]);

		rt_thread_mdelay(50);
	}
}

int gray_sensor_init(void)
{
	 drv_gray_sensor= rt_thread_create("drv_gray_sensor",
                                 gray_sensor_entry, RT_NULL,
                                 1024,
                                 11, 1);

    if (drv_gray_sensor != RT_NULL)
    {
        rt_thread_startup(drv_gray_sensor);
    }
	return 0;
}
//INIT_APP_EXPORT(gray_sensor_init);
