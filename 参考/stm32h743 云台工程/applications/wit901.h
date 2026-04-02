#ifndef __WIT901_H
#define __WIT901_H

#include <rtthread.h>
#include <rtdevice.h>

#define Acceleration 0x51;
#define	Angular 0x52;
#define Angle 0x53;
#define Magnetic 0x54;

typedef struct{
	uint8_t head;
	uint8_t buff[100];
	int16_t data[4];
	uint8_t rx_data;
}WIT901_ReceivePackageTypedef;
typedef struct{
	float Ax;
	float Ay;
	float Az;
	float Wx;
	float Wy;
	float Wz;
	float Hx;
	float Hy;
	float Hz;
	float Roll;
	float Pitch;
	float Yaw;
	float Tem;
}WIT901_Receive_DataTypedef;
typedef struct{
	WIT901_ReceivePackageTypedef receive_package;
	WIT901_Receive_DataTypedef receive_data;
}WIT901_ReceiveHandleTypedef;
typedef struct{
	WIT901_ReceiveHandleTypedef receivehandle;
}WIT901_HandleTypeDef;

extern WIT901_HandleTypeDef wit901;

void WIT901_Datain(WIT901_HandleTypeDef *wit901,uint8_t data);
void WIT901_Debug(WIT901_HandleTypeDef *wit901);

#endif



