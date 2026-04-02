#include "ik.h"
#include "meArm.h"
#include "ulog.h"
#include "SM2924_Modbus.h"
#include "gripper.h"
#include "math.h"
#include "chassis_module_mai.h"
#include "chassis.h"
#include "chassis_port.h"

float meArm_present_x=0;
float meArm_present_y=13;
float meArm_present_z=3;

#define Delay_time 250 //每个动作延时

void car_run(float x,float y,float z)
{
	uint8_t ok_flag = 0;
	chassis_pos.x_m=x;
	chassis_pos.y_m=y;
	chassis_pos.z_rad=z;
	chassis_set_pos(&chassis_mai,&chassis_pos);
	send_start=1;
	rt_thread_mdelay(50);
	
	while(!ok_flag)	//没有到位的时候就一直等待
	{
		ok_flag = motor_moveOK();
		rt_thread_mdelay(20);
	}	
}


void meArm_Init(void)
{
	meArm_goDirectlyTo(0,13,5);	//夹爪初始的位置
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	Gripper_Init(50);	//初始夹爪角度为30度
}

//完成伸臂，夹取，收臂过程
void meArm_Gripper_Get(void)
{
//	meArm_goDirectlyTo(0,13,3);
//	Motor_FTWriteSynchronous(&Motor_FT_Left);
//	Motor_FTWriteSynchronous(&Motor_FT_Right);
//	rt_thread_mdelay(Delay_time);
	
	meArm_goDirectlyTo(0,23,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	meArm_goDirectlyTo(0,23,-10);	//到达夹取位置
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	//car_runing
//	chassis_pos.x_m=0;
//	chassis_pos.y_m=0.1;
//	chassis_pos.z_rad=0;
//	chassis_set_pos(&chassis_mai,&chassis_pos);
//	send_start=1;
//	while(motor_moveOK()==0)	//没有到位的时候就一直等待
//	{
//		rt_thread_mdelay(50);
//	}
	car_run(0,0.17,0);
	
	meArm_openGripper();	//夹取
	rt_thread_mdelay(Delay_time);
	
	meArm_goDirectlyTo(0,23,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	
	meArm_goDirectlyTo(0,13,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
		//car_runningback
		car_run(0,-0.17,0);	//收回机械臂再回退
}
//放东西到1分区
void meArm_Gripper_Release_1(void)
{

	//暂时不需要
	meArm_goDirectlyTo(0,18,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	

	//car_runing
	
	
	meArm_closeGripper();	//释放
	rt_thread_mdelay(Delay_time);
	
	
	meArm_goDirectlyTo(0,13,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	//机械臂回退后再运行
	//car_run(0,-0.2,0);
}

//放东西到2分区
void meArm_Gripper_Release_2(void)
{
//	meArm_goDirectlyTo(0,13,3);
//	Motor_FTWriteSynchronous(&Motor_FT_Left);
//	Motor_FTWriteSynchronous(&Motor_FT_Right);
//	rt_thread_mdelay(Delay_time);
	
	
	meArm_goDirectlyTo(0,20,8);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	//car_running
	//抬升机械臂后再前进
	car_run(0,0.115,0);
	
	
	meArm_goDirectlyTo(0,20,4);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	meArm_closeGripper();	//释放
	rt_thread_mdelay(Delay_time);
	
	meArm_goDirectlyTo(0,13,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	//car_runningback
	car_run(0,-0.115,0);
	rt_thread_mdelay(Delay_time);
}

//放东西到3分区的第一种模式
void meArm_Gripper_Release_3_1(void)
{
//	meArm_goDirectlyTo(0,13,3);
//	Motor_FTWriteSynchronous(&Motor_FT_Left);
//	Motor_FTWriteSynchronous(&Motor_FT_Right);
//	rt_thread_mdelay(Delay_time);
	
	//车先不正对3分区
	//先伸臂
	
	meArm_goDirectlyTo(0,18,18);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
//	meArm_goDirectlyTo(0,20,17);
//	Motor_FTWriteSynchronous(&Motor_FT_Left);
//	Motor_FTWriteSynchronous(&Motor_FT_Right);
//	rt_thread_mdelay(Delay_time);
	//转向
	car_run(0,0,0.7854);
	
	//向前
	car_run(0,0.1,0);
	
	//释放夹爪
	meArm_closeGripper();	//释放
	rt_thread_mdelay(Delay_time);
	
	meArm_openGripper();	//关闭夹爪
	rt_thread_mdelay(Delay_time);
	
	//向后
	car_run(0,-0.1,0);
	//回转方向
	//car_running
	car_run(0,0,-0.7854);
	
	
	meArm_goDirectlyTo(0,13,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	rt_thread_mdelay(Delay_time);
	meArm_closeGripper();	//释放夹爪
	//rt_thread_mdelay(Delay_time);
	
}

//放东西到3分区的第二种模式
void meArm_Gripper_Release_3_2(void)
{
//	meArm_goDirectlyTo(0,13,3);
//	Motor_FTWriteSynchronous(&Motor_FT_Left);
//	Motor_FTWriteSynchronous(&Motor_FT_Right);
//	rt_thread_mdelay(Delay_time);
	
	//车先不正对3分区
	//先伸臂
	meArm_goDirectlyTo(0,18,18);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	//转向
	car_run(0,0,0.7854);
	
	//向前
	car_run(0,0.1,0);
	
	//释放夹爪
	rt_thread_mdelay(Delay_time);
	meArm_closeGripper();	//释放
	rt_thread_mdelay(Delay_time);
	
	meArm_openGripper();	//关闭夹爪
	rt_thread_mdelay(Delay_time);
	
	//向后
	car_run(0,-0.1,0);
	rt_thread_mdelay(200);
	//回转方向
	//car_running
	car_run(0,0,-0.7854);


	
	meArm_goDirectlyTo(0,13,5);
	Motor_FTWriteSynchronous(&Motor_FT_Left);
	Motor_FTWriteSynchronous(&Motor_FT_Right);
	rt_thread_mdelay(Delay_time);
	
	rt_thread_mdelay(Delay_time);
	meArm_closeGripper();	//释放

	
}
//直接走到目标位置，不需要考虑路径
//跟小车坐标一致
//Set servos to reach a certain point directly without caring how we get there 
void meArm_goDirectlyTo(float x, float y, float z) {
  float radBase,radShoulder,radElbow;
  if (solve(x, y, z, &radBase, &radShoulder, &radElbow)) {
//    _base.write(angle2pwm(_svoBase,radBase));
//    _shoulder.write(angle2pwm(_svoShoulder,radShoulder));
//    _elbow.write(angle2pwm(_svoElbow,radElbow));
//    _x = x; _y = y; _z = z;
		meArm_present_x=x;
		meArm_present_y=y;
		meArm_present_z=z;
		Motor_FTSetAngle(&Motor_FT_Left,radShoulder);
		Motor_FTSetAngle(&Motor_FT_Right,radElbow);
		LOG_D("%.3f,%.3f,%.3f",radBase,radShoulder,radElbow);
  }    
	else{
		LOG_D("mearm cannot get");
	}
}
//平滑的走某个路径到终点
//Travel smoothly from current point to another point
void meArm_gotoPoint(float x, float y, float z) {
  //Starting points - current pos
  float x0 = meArm_present_x; 
  float y0 = meArm_present_y; 
  float z0 = meArm_present_z;
  float dist = sqrt((x0-x)*(x0-x)+(y0-y)*(y0-y)+(z0-z)*(z0-z));
  int step = 2;
  for (int i = 0; i<dist; i+= step) {
    meArm_goDirectlyTo(x0 + (x-x0)*i/dist, y0 + (y-y0) * i/dist, z0 + (z-z0) * i/dist);
    //延时
		LOG_D("x:%.3f,y:%.3f,z:%.3f",meArm_present_x,meArm_present_y,meArm_present_z);
		rt_thread_mdelay(500);
  }
  meArm_goDirectlyTo(x, y, z);
  rt_thread_mdelay(500);
}
//从极坐标到笛卡尔坐标系
//Get x and y from theta and r
void meArm_polarToCartesian(float theta, float r, float *x, float  *y){
//    _r = r;
//    _t = theta;
//    x = r*sin(theta);
//    y = r*cos(theta);
}
//路径规划走到圆柱型极坐标系
//Same as above but for cylindrical polar coodrinates
void meArm_gotoPointCylinder(float theta, float r, float z){
    float x, y;
    meArm_polarToCartesian(theta, r, &x, &y);
    meArm_gotoPoint(x,y,z);
}
//直接走到圆柱形极坐标系某点
void meArm_goDirectlyToCylinder(float theta, float r, float z){
    float x, y;
    meArm_polarToCartesian(theta, r, &x, &y);
    meArm_goDirectlyTo(x,y,z);
}
//检查是否可达
//Check to see if possible
uint8_t meArm_isReachable(float x, float y, float z) {
  float radBase,radShoulder,radElbow;
  return (solve(x, y, z, &radBase, &radShoulder, &radElbow));
}
//夹取东西
//Grab something
void meArm_openGripper() {
	Gripper_Set_Angle(92);
}
//释放
//Let go of something
void meArm_closeGripper() {
	Gripper_Set_Angle(70);
}

////Current x, y and z
//float meArm_getX() {
//  //return _x;
//}
//float meArm_getY() {
//  //return _y;
//}
//float meArm_getZ() {
//  //return _z;
//}


//float meArm_getR() {
//  //return _r;
//}
//float meArm_getTheta() {
//  //return _t;
//}
