/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"
#include "adc.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

int turn_cnt=10000;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=300;	


int Pos_Left,Pos_Right;		//脉冲数
int start_cnt_left,start_cnt_right;	//脉冲数

float turn_p=0.0008f;	//转向环的系数
int Run_Mode=0;//小车运行模式	//0停车，1巡线 2开环旋转
uint8_t round_number=1;	//走的总圈数
uint8_t turn_number=0;	//旋转的次数（拐点个数）

float fllow_speed=0.22;	//巡线时的直线速度


int now_mid_value=0;				//当前中线值
int last_mid_value=0;				//上次中线值
int last_last_mid_value=0;	//上上次中线值
//电机旋转一周780
//旋转时期望的左右电机脉冲数
//uint32_t target_left_number=-300;
//uint32_t target_right_number=6000;
//旋转时期望的旋转时间（ms）
//uint16_t target_time=2000;
//z大于0 向左旋转
//z小于0 向右旋转
//5ms中断
void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
					turn_cnt++;

					Key();
					LED_Flash(100);
					//测试代码
//					if(Run_Mode==0)
//					{
//						Set_PWM(0,0);	//停车
//					}
//					else
//					{
//						static uint16_t test_count=0;
//						test_count++;
//						if(test_count<=500)
//						{
//							 unsigned short Normal[8];
//							if(Get_Normalize_For_User(&sensor,Normal)){
//								//printf("Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
//									last_last_mid_value=last_mid_value;
//									last_mid_value=now_mid_value;
//									now_mid_value=gray_count_value(Normal);
//									now_mid_value=(int)(0.7*now_mid_value+0.2*last_mid_value+0.1*last_last_mid_value);
//									Move_X=0.3;
//									//Move_Z=-now_mid_value*turn_p;
//							}
//						}
//						else if(test_count>500&&test_count<1000)
//						{
//							
//							 unsigned short Normal[8];
//							if(Get_Normalize_For_User(&sensor,Normal)){
//								//printf("Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
//									last_last_mid_value=last_mid_value;
//									last_mid_value=now_mid_value;
//									now_mid_value=gray_count_value(Normal);
//									now_mid_value=(int)(0.7*now_mid_value+0.2*last_mid_value+0.1*last_last_mid_value);
//									Move_X=-0.3;
//									//Move_Z=now_mid_value*turn_p;
//								}
//							
//						}
//						else if(test_count>=1000)
//						{
//							test_count=0;
//						}
//								Get_Target_Encoder(Move_X,Move_Z);
//								Pos_Left+=Get_Encoder_countA;
//								Pos_Right+=Get_Encoder_countB;
//								Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);
//								Get_Encoder_countA=Get_Encoder_countB=0;
//															////			//计算左右电机对应的PWM
//								MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
//								MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
//								//Limit
//								Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
//				}
						//测试代码
						
					//停车
					if(Run_Mode==0)
					{
						//Move_X = Move_Z = 0;
						Set_PWM(0,0);	//停车
					}
					//巡线
					if(Run_Mode==1)
					{
							unsigned short Normal[8];
							if(Get_Normalize_For_User(&sensor,Normal)){
								//printf("Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
									last_last_mid_value=last_mid_value;
									last_mid_value=now_mid_value;
									now_mid_value=gray_count_value(Normal);
									now_mid_value=(int)(0.7*now_mid_value+0.2*last_mid_value+0.1*last_last_mid_value);
									Move_X=fllow_speed;
									Move_Z=-now_mid_value*turn_p;
							}
					}
					else if(Run_Mode==2)//开环转弯
					{
							static uint16_t time_count=0;
							static uint8_t last_flag=0;	//最后一次检测标志位
							time_count++;
						if(last_flag==1)
						{
							Run_Mode=0;	//停车
							last_flag=0;
						}

						if(time_count>=100)	//160
						{
							uint16_t Normal[8];
							if(Get_Normalize_For_User(&sensor,Normal))//获取灰度值
							{
								if(Normal[1]<2300||Normal[2]<=2300||Normal[3]<=2300||Normal[4]<2300)
								{
									Run_Mode=1;
									turn_cnt=0;
									time_count=0;
									turn_number++;		//旋转次数
									if(turn_number==round_number*4)	//到达
									{
										//Run_Mode=0;	//停车
										last_flag=1;
									}
								}
							}	
						}

					}

					if(Run_Mode==1)	//巡线模式
					{
								Get_Target_Encoder(Move_X,Move_Z);
								Pos_Left+=Get_Encoder_countA;
								Pos_Right+=Get_Encoder_countB;
								Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);
								Get_Encoder_countA=Get_Encoder_countB=0;
															////			//计算左右电机对应的PWM
								MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
								MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
								//Limit
								Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
								//Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
							}
							else if(Run_Mode==2)//转弯模式
							{
								MotorA.Target_Encoder=0.05;		//两个电机相反会疯转
								MotorB.Target_Encoder=0.25;
								Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);
								Get_Encoder_countA=Get_Encoder_countB=0;
															////			//计算左右电机对应的PWM
								MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
								MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
								//Limit
								Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);

						
						}
				}
			}
}
		

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	float Encoder_A_pr,Encoder_B_pr; 
	OriginalEncoder.A=-Encoder1;	
	OriginalEncoder.B=-Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;
	//编码器原始数据转换为车轮速度，单位m/s
	MotorA.Current_Encoder= Encoder_A_pr*Frequency*Perimeter/780.0f;  
	MotorB.Current_Encoder= Encoder_B_pr*Frequency*Perimeter/780.0f;   //1560=2*13*30=2（两路脉冲）*1（上升沿计数）*霍尔编码器13线*电机的减速比
	
}
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //Wheel target speed limit //车轮目标速度限幅
	if(Vx<0) Vz=-Vz;
	else     Vz=Vz;
	//Inverse kinematics //运动学逆解
	 MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //计算出左轮的目标速度
	 MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //计算出右轮的目标速度
	//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
//	 MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	 MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
}


/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压高于7V且使能开关打开
//	{
//		temp = 1;
//	}
	return temp;			
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	float bais_li=+Velocity_KI*Bias;
		if(bais_li>1000)bais_li=1000;
	if(bais_li<-1000)bais_li=-1000;
	 Pwm+=Velocity_KP*(Bias-Last_bias)+bais_li;   	//增量式PI控制器
	

	 if(Pwm>6000)Pwm=6000;
	 if(Pwm<-6000)Pwm=-6000;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	
	
	
	float bais_li=+Velocity_KI*Bias;
		if(bais_li>1000)bais_li=1000;
	if(bais_li<-1000)bais_li=-1000;
	 Pwm+=Velocity_KP*(Bias-Last_bias)+bais_li;   	//增量式PI控制器
	
	
	 if(Pwm>6000)Pwm=6000;
	 if(Pwm<-6000)Pwm=-6000;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
//	}
	
//	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
//	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/200;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Get_Target_Encoder(Move_X,Move_Z);
}

/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=key_scan(200);//1：单击，2：双击 3：长按
	if(tmp==1)				//当前修改值加一个刻度
	{
		switch(page_number)	//
		{
			case 0:round_number++;	//调整圈数
						turn_number=0;		//清空转弯数
						if(round_number>5)	//单击设置圈数
						{
							
							round_number=1;
						}
						break;
			case 1:is_jianche=1;	//开启检测
						break;
			case 2:turn_p+=0.00005f;	//转向环kp增大0.00005f
									break;
			case 3:fllow_speed+=0.01;	//直线速度增加0.01
									break;
			case 4:jiaozhun=1;
						break;
			default:break;
		}
//		round_number++;
//		if(round_number>5)	//单击设置圈数
//		{
//			round_number=1;
//		}
	}		
	else if(tmp==2&&page_number==0)	//在圈数选择界面双击运行
	{
		Run_Mode=1;
	}

	//按键2
		tmp=key_scan_2(200);
		if(tmp==1)	//当前修改值减小一个刻度
		{
			switch(page_number)
			{
					case 0:turn_number=0;	//清空转弯数
						if(round_number==1)	//，为1时，再减1值为0，单击设置圈数
					{
							round_number=5;
					}
					else{
							round_number--;
					}
					break;
					case 1:is_jianche=0;	//关闭检测
									break;
					case 2:turn_p-=0.00005f;	//转向环kp减小0.00005f
									break;
					case 3:fllow_speed-=0.01;	//直线速度减小0.01
									break;
					case 4:jiaozhun=2;		//校准黑色
									break;
			}

		}
		else if(tmp==2)	//双击运行,切换菜单
		{
			page_number++;
			if(page_number>4)
			{
				page_number=0;
			}
		}
	
}
