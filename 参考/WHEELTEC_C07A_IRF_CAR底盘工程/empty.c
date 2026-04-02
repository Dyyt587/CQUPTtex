/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
u8 Car_Mode=Diff_Car;
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的
u8 PID_Send;            //延时和调参相关变量
float RC_Velocity=200,RC_Turn_Velocity,Move_X,Move_Y,Move_Z,PS2_ON_Flag;               //遥控控制的速度
float Velocity_Left,Velocity_Right; //车轮速度(mm/s)
u16 test_num,show_cnt;
float Voltage=0;



unsigned short Anolog[8]={0};
unsigned short white[8]={3048,3060,3010,2970,3070,3100,3090,2880};
unsigned short black[8]={480,760,1027,860,1210,1120,1335,770};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};


int main(void)
{
    // 系统初始化
    SYSCFG_DL_init();  // 初始化系统配置
    // 清除所有外设的中断挂起状态
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);    // 编码器A中断
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);    // 编码器B中断
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);  // UART0串口中断
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);  // UART1串口中断
    // 使能各外设的中断
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);    // 开启编码器A中断
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);    // 开启编码器B中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN); // 开启UART0中断
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN); // 开启UART1中断
    // 定时器和ADC相关中断配置
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);  // 清除定时器0中断挂起
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);        // 开启定时器0中断
	
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);
	
		//初始化传感器，不带黑白值
		No_MCU_Ganv_Sensor_Init_Frist(&sensor);
		No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		Get_Anolog_Value(&sensor,Anolog);
	//此时打印的ADC的值，可用通过这个ADC作为黑白值的校准
	//也可以自己写按键逻辑完成一键校准功能
//		sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
//		uart0_send_string((char *)rx_buff);
		delay_ms(100);
		memset(rx_buff,0,256);
	//得到黑白校准值之后，初始化传感器
		No_MCU_Ganv_Sensor_Init(&sensor,white,black);
		//Set_PWM(1,1);
		//delay_ms(100);
    OLED_Init();  // 初始化OLED显示屏
		NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);  // 清除定时器0中断挂起
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);        // 开启定时器0中断
	
    // 主循环
    while (1) 
    {
			if(page_number==4)
			{
				if(jiaozhun==1)	//校准白色
				{
					No_MCU_Ganv_Sensor_Init_Frist(&sensor);
					No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
					Get_Anolog_Value(&sensor,Anolog);
					memcpy(white,Anolog,sizeof(Anolog));
					oled_show();         //  OLED显示更新
					delay_ms(500);
					memset(rx_buff,0,256);
				//得到黑白校准值之后，初始化传感器
					No_MCU_Ganv_Sensor_Init(&sensor,white,black);
					jiaozhun=0;
				}
				if(jiaozhun==2)	//校准白色
				{
					No_MCU_Ganv_Sensor_Init_Frist(&sensor);
					No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
					Get_Anolog_Value(&sensor,Anolog);
					memcpy(black,Anolog,sizeof(Anolog));
					oled_show();         //  OLED显示更新
					delay_ms(500);
					memset(rx_buff,0,256);
				//得到黑白校准值之后，初始化传感器
					No_MCU_Ganv_Sensor_Init(&sensor,white,black);
					jiaozhun=0;
				}
			}
						//无时基传感器常规任务，包含模拟量，数字量，归一化量
			No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
			//有时基传感器常规任务，包含模拟量，数字量，归一化量
//			No_Mcu_Ganv_Sensor_Task_With_tick(&sensor)
			//获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
//			Digtal=Get_Digtal_For_User(&sensor);
//			sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
//			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			
			//获取传感器模拟量结果(有黑白值初始化后返回1 没有返回 0)
			if(Get_Anolog_Value(&sensor,Anolog)){
			//printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);

			}
			
			//获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)
			if(Get_Normalize_For_User(&sensor,Normal)){
				//printf("Normalize %d,%d,%d,%d,%d,%d,%d,%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);

			}
			//printf("white %d,%d,%d,%d,%d,%d,%d,%d\r\n",white[0],white[1],white[2],white[3],white[4],white[5],white[6],white[7]);
			//printf("black %d,%d,%d,%d,%d,%d,%d,%d\r\n",black[0],black[1],black[2],black[3],black[4],black[5],black[6],black[7]);
			uint8_t result=gray_is_turn(Normal);
			//printf("resu:%d %d\n",result,Run_Mode);
			extern int turn_cnt;
			if(result!=0&&Run_Mode==1 )	//切换运行模式
			{
				if((turn_cnt)>200) Run_Mode=2;
			}
			else	
			{
				//Run_Mode=1;
			}
			//经典版理论性能1khz，只需要delay1ms，青春版100hz，需要delay10ms，否则不能正常使用
			
				Voltage = result;		//采样小车当前拐点
        BTBufferHandler();    // 处理蓝牙数据缓冲区
        oled_show();         //  OLED显示更新
        //APP_Show();          //  APP显示处理
			//printf("left:%d,%d\n",Get_Encoder_countA,Get_Encoder_countB);
			//Set_PWM(2000,2000);
			delay_ms(10);
			
    }
}



