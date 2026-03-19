#include "SEN0366_Laser.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "ulog.h"
#include "string.h"


//激光测距模块
//串口7
//采用中断模式逐个解析


#define USART7_NAME  "uart7"
static rt_device_t serial7;										//串口设备句柄
static rt_thread_t thread_laser = RT_NULL;  

static struct serial_configure uart7_config = RT_SERIAL_CONFIG_DEFAULT;  //初始化串口配置参数

Sen0366_LaserHandle_t sen0366_laser;	//laser结构体
/**
*  @brief sen0366_laser激光测距初始化
*  @param laser:激光测距句柄
*					
*/
void Sen0366_Laser_Init(Sen0366_LaserHandle_t *laser)
{
	laser->head[0]=0x80;	//初始化头部
	laser->head[1]=0x06;
	laser->head[2]=0x83;

	
	laser->index=0;
	laser->check=0;
	laser->status=0;
	
	//rt_device_write(serial7,0,laser->head,4);
}
/**
*  @brief sen0366_laser激光测距数据帧处理
*  @param laser:激光测距句柄
*					data:接收到的单个数据
*	 @retval uint8_t 接收数据状态，0:数据解析未成功，1:数据成功解析				
*/
uint8_t  Sen0366_Laser_Handle(Sen0366_LaserHandle_t *laser,uint8_t data)
{
	if(laser->index==0)
	{
			laser->raw_buff[0]=laser->raw_buff[1];
			laser->raw_buff[1]=laser->raw_buff[2];
			laser->raw_buff[2]=data;
			if(laser->raw_buff[0]==laser->head[0]&&\
				laser->raw_buff[1]==laser->head[1]&&\
			laser->raw_buff[2]==laser->head[2]) //帧头校验合格
			{
				laser->index=3;
				return 0;
			}
			return 0;	//未接收到数据头，返回
			
	}
	laser->raw_buff[laser->index]=data;
	laser->index++;
	if(laser->index==11)	//接收完所有数据
	{
		laser->index=0;	//清零索引
		laser->check=0;	//清零校验值
		for(uint8_t i=0;i<10;i++)
		{
			laser->check=laser->check+laser->raw_buff[i];
		}
		laser->check=~laser->check+1;
		if(laser->check==laser->raw_buff[10])
		{
			if(laser->raw_buff[3]=='E'&&laser->raw_buff[4]=='R'&&laser->raw_buff[5]=='R')	//数据错误信息
			{
				LOG_D("Sen0366_Laser Distance Error");

				memset(laser->raw_buff,0,20);	//清空raw_buff
				return 0;
			}
			else
			{
				laser->distance=(laser->raw_buff[3]-0x30)*100+(laser->raw_buff[4]-0x30)*10+(laser->raw_buff[5]-0x30)*1+\
				(laser->raw_buff[7]-0x30)*0.1+(laser->raw_buff[8]-0x30)*0.01+(laser->raw_buff[9]-0x30)*0.001;
				memset(laser->raw_buff,0,20);	//清空raw_buff
				//LOG_D("dis:%.3f",laser->distance);
				return 1;
			}
		}
		else	//校验不通过
		{
			LOG_D("check rror");
			memset(laser->raw_buff,0,20);	//清空raw_buff
			return 0;
		}
	}
	return 0;
}

//串口接收回调函数
static rt_err_t usart_rx_callback(rt_device_t dev, rt_size_t size)
{

	//LOG_D("size:%d",size);
	uint8_t rx_buff[size];
	
	rt_device_read(dev,0,rx_buff,size);
	//LOG_D("%s",rx_buff);
	for(uint8_t i=0;i<size;i++)
	{
		if(Sen0366_Laser_Handle(&sen0366_laser,rx_buff[i])==1)	//成功接收激光测距模块数据
		{
			LOG_D("distance:%.3f",sen0366_laser.distance);
		}
	}
	return RT_EOK;
}

//线程运行函数
static void thread_laser_entry(void *parameter)
{
	uint8_t rx_data=0;

	rt_err_t result;
	static uint8_t rx_buffer[100];


	rt_device_open(serial7, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);	//再打开串口
		    /* 设置接收回调函数 */
   rt_device_set_rx_indicate(serial7, usart_rx_callback);	
	
	Sen0366_Laser_Init(&sen0366_laser);	//激光测距初始化
	
	rt_thread_mdelay(50);
	while(1)
	{
	
    rt_thread_mdelay(10);	//延时太长会导致数据读出慢
	}
}

int laser1_init(void) {

  thread_laser =
      rt_thread_create("thread_laser", thread_laser_entry, RT_NULL, 2048, 10, 1);
	
    serial7 = rt_device_find(USART7_NAME);
    if (!serial7)
    {
        LOG_D("find %s failed!\n",USART7_NAME);
        return RT_ERROR;
    }
				//修改串口配置参数 
		uart7_config.baud_rate = BAUD_RATE_9600;        // 修改波特率为 9600
		uart7_config.data_bits = DATA_BITS_8;           // 数据位 8
		uart7_config.stop_bits = STOP_BITS_1;           // 停止位 1
		uart7_config.rx_bufsz     = BSP_UART7_RX_BUFSIZE;       // 修改缓冲区 
		uart7_config.tx_bufsz			= BSP_UART7_TX_BUFSIZE;
		uart7_config.parity    = PARITY_NONE;           // 无奇偶校验位
		
		if(rt_device_control(serial7, RT_DEVICE_CTRL_CONFIG, &uart7_config)!=RT_EOK)
		{
			LOG_D("Laser Change BAUD ERROR");
		}
		if (thread_laser!= RT_NULL) {
			rt_thread_startup(thread_laser);
		}

  return 0;
}
//INIT_APP_EXPORT(laser1_init);