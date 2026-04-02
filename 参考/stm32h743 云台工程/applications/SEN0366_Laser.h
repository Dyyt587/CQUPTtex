#ifndef __SEN0366_H
#define  __SEN0366_H

#include <rtthread.h>

typedef struct
{
	uint8_t head[4];			//帧头,0x80,0x06,0x03,0x77
	uint8_t raw_buff[20];	//存储buff
	uint8_t index;				//指示当前索引
	uint8_t check;				//校验码
	uint8_t status;				//状态
	float distance;				//距离
	
}Sen0366_LaserHandle_t;
	



#endif

