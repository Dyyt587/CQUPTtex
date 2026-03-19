#ifndef __USART_DATAPARSIRING_H
#define __USART_DATAPARSIRING_H

#include <rtthread.h>
#include <rtdevice.h>



// 数据buff结构体定义
typedef struct 
{
    float float_data[10];               // 浮点数据数组
    uint32_t uint32_data[10];           // 无符号整型数据存储
    int32_t int32_data[10];             // 有符号整型数据存储
    
    uint8_t float_data_lenght;          // 浮点数数据长度
    uint8_t uint32_data_lenght;         // 无符号整型数据长度
    uint8_t int32_data_lenght;          // 有符号整型数据长度
}Data_Buff_Typedef;

// 数据帧结构体定义
typedef struct             
{
		uint8_t statue;												//解析进行的状态
    uint8_t frame_string[100];            // 数据帧有效内容
    uint8_t frame_string_copy[100];       // 数据帧有效内容双缓存
    uint8_t frame_start;                // 数据帧帧头
    uint8_t frame_lenght;               // 数据帧长度
    uint8_t frame_end;                  // 数据帧帧尾
    uint8_t data_string[10];              // 数据有效内容
    uint8_t data_start;                 // 数据帧头
    uint8_t data_lenght;                // 数据长度
    uint8_t data_end;                   // 数据帧尾
    Data_Buff_Typedef data_buff;  			// 数据buff结构体
}Data_Parsiring_Typedef;								//一个数据解析的结构体

extern Data_Parsiring_Typedef camera_data;

void Usart_Dataparsiring_Init(Data_Parsiring_Typedef *data);
void Usart_Dataparsiring_Handle(Data_Parsiring_Typedef *data,uint8_t rx_data);
void Usart_Dataparsiring(Data_Parsiring_Typedef *data);
void Usart_Dataparising_Debug(Data_Parsiring_Typedef *data);

#endif


