#include "wit901.h"
#include "ulog.h"
#include "string.h"

//wit901陀螺仪
//使用串口5


WIT901_HandleTypeDef wit901;

static uint8_t check_data(uint8_t *data,uint8_t len)
{
	uint8_t sum=0;
	uint8_t i=0;
	for(;i<len;i++)
	{
		sum+=*(data+i);
	}
	return sum;
}
//以单个数据的方式输入
void WIT901_Datain(WIT901_HandleTypeDef *wit901,uint8_t data)
{
	static uint8_t index=0;
	WIT901_ReceivePackageTypedef *receive_package=&(wit901->receivehandle.receive_package);
	WIT901_Receive_DataTypedef *receive_data=&(wit901->receivehandle.receive_data);
	receive_package->buff[index++]=data;
	if(receive_package->buff[0]!=0x55)
	{
		index--;
		memcpy(receive_package->buff,&receive_package->buff[1],index);
		return;
	}
	if(index>=11)
	{
		//HAL_UART_Transmit(&huart1,receive_package->buff,11,10);
		if(check_data(receive_package->buff,10)==receive_package->buff[10])
		{
			receive_package->data[0]=((short)receive_package->buff[2]|((short)receive_package->buff[3]<<8));
			receive_package->data[1]=((short)receive_package->buff[4]|((short)receive_package->buff[5]<<8));
			receive_package->data[2]=((short)receive_package->buff[6]|((short)receive_package->buff[7]<<8));
			receive_package->data[3]=((short)receive_package->buff[8]|((short)receive_package->buff[9]<<8));
			index=0;
			switch(receive_package->buff[1])
			{
			  case 0x51:
				{
					receive_data->Ax=receive_package->data[0]*9.8*16/32768;
					receive_data->Ay=receive_package->data[1]*9.8*16/32768;
					receive_data->Az=receive_package->data[2]*9.8*16/32768;
				  receive_data->Tem=receive_package->data[3]/100; 
				  break;
				}
				case 0x52:
				{
					receive_data->Wx=receive_package->data[0]*2000/32768.0;
					receive_data->Wy=receive_package->data[1]*2000/32768.0;
					receive_data->Wz=receive_package->data[2]*2000/32768.0;
					break;
				}
				case 0x53:
				{
					receive_data->Roll=(receive_package->data[0]*180/32768.0);
					receive_data->Pitch=(receive_package->data[1]*180/32768.0);
					receive_data->Yaw=(receive_package->data[2]*180/32768.0);
					break;
				}
				case 0x54:
				{
					receive_data->Hx=receive_package->data[0];
					receive_data->Hy=receive_package->data[1];
					receive_data->Hz=receive_package->data[2];
				  receive_data->Tem=receive_package->data[3]; 
					break;
				}
			}
			
		}
		else
		{
			 index--;
			 memcpy(receive_package->buff,&receive_package->buff[1],index);
			 return;
		}
	}
}
//数据解析
//用dma时，解析数据的函数
//默认接到44个，完整的一帧数据
//void WIT901_Parsing(WIT901_HandleTypeDef *wit901,uint8_t *data)
//{
//	
//}

void WIT901_Debug(WIT901_HandleTypeDef *wit901)
{
	WIT901_Receive_DataTypedef *data=&(wit901->receivehandle.receive_data);
//	printf("%.2f,%.2f,%.2f\n",data->Roll,data->Pitch,data->Yaw);
	//LOG_D("WIT:%.2f,%.2f,%.2f",data->Wx,data->Wy,data->Wz);
	//LOG_D("WIT:%.2f,%.2f,%.2f",data->Ax,data->Ay,data->Az);
	LOG_D("WIT:%.2f,%.2f,%.2f",data->Roll,data->Pitch,data->Yaw);
}

static rt_thread_t thread_wit901 = RT_NULL;  
#define USART_NAME  "uart5"
static rt_device_t serial5;										//串口设备句柄
static struct rt_semaphore wit901_rx_sem;    /* 用于接收消息的信号量 */

/* 串口接收消息结构 */
struct wit901_rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

/* 消息队列控制块 */
static struct rt_messagequeue wit901_rx_mq;	//wit901接收消息队列

//线程运行函数
static void thread_wit901_entry(void *parameter)
{
	uint8_t rx_data=0;
	struct wit901_rx_msg msg;
	rt_err_t result;
	static uint8_t rx_buffer[1024];
	while(1)
	{
	      
        /* 从消息队列中读取消息 */
        result = rt_mq_recv(&wit901_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result > 0)
        {
            /* 从串口读取数据 */
            rt_device_read(msg.dev, 0, rx_buffer, msg.size);
						//LOG_D("%d",msg.size);
						for(uint16_t i=0;i<msg.size;i++)
						{
							 WIT901_Datain(&wit901,rx_buffer[i]);	//将数据放入解析队列
						}
						rt_memset(&msg, 0, sizeof(msg));	//清零接收数据标志位
						WIT901_Debug(&wit901);	//将解析完成的数据打印出来
        }
    rt_thread_mdelay(3);	//延时太长会导致数据读出慢
	}
}
//串口接收回调函数
static rt_err_t usart_rx_callback(rt_device_t dev, rt_size_t size)
{
//	uint8_t rx_data[100]={0};
//	rt_device_read(serial5,0,(uint8_t *)rx_data,size);
//	LOG_D("u5:%d",size);
	  struct wit901_rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;
    result = rt_mq_send(&wit901_rx_mq, &msg, sizeof(msg));	//发送消息队列
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        LOG_D("usart5 message queue full!");
    }
    return result;
	
	//return RT_EOK;
}
int wit901_init(void) {

  thread_wit901 =
      rt_thread_create("thread_usart", thread_wit901_entry, RT_NULL, 1024, 10, 1);
	
	    /* 初始化消息队列 */
		static char msg_pool[256];
    rt_mq_init(&wit901_rx_mq, "wit901_rx_mq",
               msg_pool,                 /* 存放消息的缓冲区 */
               sizeof(struct wit901_rx_msg),    /* 一条消息的最大长度 */
               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
	    /* 查找串口设备 */
    serial5 = rt_device_find(USART_NAME);
    if (!serial5)
    {
        LOG_D("find %s failed!\n",USART_NAME);
        return RT_ERROR;
    }
		rt_device_open(serial5, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
		    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial5, usart_rx_callback);
		
		if (thread_wit901 != RT_NULL) {
			rt_thread_startup(thread_wit901);
		}

  return 0;
}
//INIT_APP_EXPORT(wit901_init);