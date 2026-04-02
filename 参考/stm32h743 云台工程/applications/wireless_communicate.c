#include "wireless_communicate.h"
#include "ulog.h"
#include "usart_dataparsiring.h"

//无线透传dl_22
//串口7

Data_Parsiring_Typedef wireless_data;	//解析数据结构体


static rt_thread_t thread_wireless = RT_NULL;  
#define USART_NAME  "uart7"
static rt_device_t serial7;										//串口设备句柄


/* 串口接收消息队列的消息结构 */
struct wireless_rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
static struct rt_messagequeue wireless_rx_mq;	//wireless接收消息队列

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
//串口接收回调函数
static rt_err_t usart_rx_callback(rt_device_t dev, rt_size_t size)
{
	  struct wireless_rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;
    result = rt_mq_send(&wireless_rx_mq, &msg, sizeof(msg));	//发送消息队列
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        LOG_D("usart7 message queue full!");
    }
    return result;
	//return RT_EOK;
}

//线程运行函数
static void thread_wireless_entry(void *parameter)
{
	uint8_t rx_data=0;
	struct wireless_rx_msg msg;
	rt_err_t result;
	static uint8_t rx_buffer[1024];
	Usart_Dataparsiring_Init(&wireless_data);	//先进行数据的解析结构体初始化

	rt_device_open(serial7, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);	//再打开串口
		    /* 设置接收回调函数 */
   rt_device_set_rx_indicate(serial7, usart_rx_callback);	
	rt_thread_mdelay(50);
	while(1)
	{
		result = rt_mq_recv(&wireless_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
		if (result > 0)
		{
            /* 从串口读取数据 */
         rt_device_read(msg.dev, 0, rx_buffer, msg.size);
				 LOG_D("%s",rx_buffer);
				for(uint16_t i=0;i<msg.size;i++)
				{
						//LOG_D("%c",rx_buffer[i]);
					Usart_Dataparsiring_Handle(&wireless_data,rx_buffer[i]);//数据处理
				}
				rt_memset(rx_buffer,0,msg.size);	//清空接收buffer
				rt_memset(&msg, 0, sizeof(msg));	//清零接收数据标志位
				//数据利用
				Usart_Dataparising_Debug(&wireless_data);	//进行数据Debug

    }
    rt_thread_mdelay(10);	//延时太长会导致数据读出慢
	}
}

int wireless_init(void) {

  thread_wireless =
      rt_thread_create("thread_wireless", thread_wireless_entry, RT_NULL, 1024, 10, 1);
	
	    /* 初始化消息队列 */
		static char msg_pool[1024];
    rt_mq_init(&wireless_rx_mq, "wireless_rx_mq",
               msg_pool,                 					/* 存放消息的缓冲区 */
               sizeof(struct wireless_rx_msg),    /* 一条消息的最大长度 */
               sizeof(msg_pool),        				 /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
	    /* 查找串口设备 */
    serial7 = rt_device_find(USART_NAME);
    if (!serial7)
    {
        LOG_D("find %s failed!\n",USART_NAME);
        return RT_ERROR;
    }

		
		if (thread_wireless!= RT_NULL) {
			rt_thread_startup(thread_wireless);
		}

  return 0;
}
//INIT_APP_EXPORT(wireless_init);














