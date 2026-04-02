#include "SM2924_Modbus.h"

//PD4--DE
//FT_SM2924_HandleTypeDef Motor_FT2;
//FT_SM2924_HandleTypeDef Motor_FT3;
//FT_SM2924_HandleTypeDef Motor_FT4;
FT_SM2924_HandleTypeDef Motor_FT_Left;
FT_SM2924_HandleTypeDef Motor_FT_Right;


extern USHORT   usMRegInStart                              ;
extern USHORT   usMRegInBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_INPUT_NREGS];
//Master mode:HoldingRegister variables
extern USHORT   usMRegHoldStart                            ;
extern USHORT   usMRegHoldBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];

//uint16_t Motor_FTReadBuf(uint16_t id,uint16_t addr)
//{
//	return usMRegHoldBuf[id-1][addr];
//}

/**
*  @brief 飞特电机初始化
*  @param motor:电机句柄
*					id:电机对应id
*/
void Motor_FTInit(FT_SM2924_HandleTypeDef * motor,uint8_t id)
{
	motor->id=id;
	motor->pos_offset=0;
	motor->polar=1;	//极性默认为1
	
	motor->control.pos=0;
	motor->control.torque_enable=0;
	motor->control.speed=speed_default;
	motor->control.accelerate=accel_default;
	motor->control.torque_limit=torque_limit_default;

	motor->statues.pos=0;
	motor->statues.duty=0;
	motor->statues.is_moving=0;
	motor->statues.speed=0;
	motor->statues.temp=0;
	motor->statues.voltage=0;
}
/**
*  @brief 飞特电机设置电机位置偏移
*  @param motor:电机句柄
*					offset:偏移量
*/
void Motor_FTSetPosOffSet(FT_SM2924_HandleTypeDef * motor,uint16_t offset)
{
	motor->pos_offset=offset;
}
/**
*  @brief 飞特电机设置电机极性(方向)
*  @param motor:电机句柄
*					polar:极性
*/
void Motor_FTSetPolar(FT_SM2924_HandleTypeDef * motor,int8_t polar)
{
	motor->polar=polar;
}
/**
*  @brief 飞特电机设置电机位置
*  @param motor:电机句柄
*					pos:电机位置
*/
void Motor_FTSetPos(FT_SM2924_HandleTypeDef * motor,int16_t pos)
{
	motor->control.pos=pos;
}
/**
*  @brief 飞特电机设置电机的角度
*  @param motor:电机句柄
*					angle:角度(弧度制)
*/
void Motor_FTSetAngle(FT_SM2924_HandleTypeDef * motor,float angle)
{
	float param=2048/ft_pi;	//计算比例
	motor->control.pos=param*angle;
}
/**
*  @brief 飞特电机设置电机的角度
*  @param motor:电机句柄
*			
*/
void Motor_FTGetAngle(FT_SM2924_HandleTypeDef * motor)
{
	//motor->control.pos=param*angle;
	float param=2048/ft_pi;	//计算比例
	motor->statues.angle=motor->statues.pos/param;
}
/**
* @brief 飞特舵机读取参数
* @param motor:电机句柄
	* @retval error_code:MODBUS错误码
*/
eMBMasterReqErrCode Motor_FTReadSynchronous(FT_SM2924_HandleTypeDef * motor)
{

	eMBMasterReqErrCode error_code = MB_MRE_NO_ERR;
	error_code=eMBMasterReqReadHoldingRegister(motor->id,SM_FIRMWARE_VERSION,4,100);
	error_code=eMBMasterReqReadHoldingRegister(motor->id,SM_DEVICE_ID,12,100);
	error_code=eMBMasterReqReadHoldingRegister(motor->id,SM_TARGET_POSITION,7,100);
	error_code=eMBMasterReqReadHoldingRegister(motor->id,SM_SERVO_STATUS,8,100);
	error_code=eMBMasterReqReadHoldingRegister(motor->id,SM_MOVE_THRESHOLD,23,100);

	motor->statues.servo_status=usMRegHoldBuf[motor->id-1][SM_SERVO_STATUS];
	motor->statues.pos=(usMRegHoldBuf[motor->id-1][SM_CURRENT_POSITION]-motor->pos_offset)*motor->polar;
	motor->statues.speed=usMRegHoldBuf[motor->id-1][SM_CURRENT_SPEED]*motor->polar;
	motor->statues.duty=usMRegHoldBuf[motor->id-1][SM_OUTPUT_PWM]*0.1;
	motor->statues.voltage=usMRegHoldBuf[motor->id-1][SM_INPUT_VOLTAGE]*0.1;
	motor->statues.temp=usMRegHoldBuf[motor->id-1][SM_INTERNAL_TEMP];
	motor->statues.is_moving=usMRegHoldBuf[motor->id-1][SM_MOVING_FLAG];
	motor->statues.current=usMRegHoldBuf[motor->id-1][SM_CURRENT_LOAD]*6.5;
	Motor_FTGetAngle(motor);//pos转换为angle
	return error_code;
}

/**
* @brief 飞特舵机读取有效的参数
* @param motor:电机句柄
	* @retval error_code:MODBUS错误码
*/
eMBMasterReqErrCode Motor_FTReadValid(FT_SM2924_HandleTypeDef * motor)
{

	eMBMasterReqErrCode error_code = MB_MRE_NO_ERR;

	error_code=eMBMasterReqReadHoldingRegister(motor->id,SM_SERVO_STATUS,8,100);

	motor->statues.servo_status=usMRegHoldBuf[motor->id-1][SM_SERVO_STATUS];
	motor->statues.pos=(usMRegHoldBuf[motor->id-1][SM_CURRENT_POSITION]-motor->pos_offset)*motor->polar;
	motor->statues.speed=usMRegHoldBuf[motor->id-1][SM_CURRENT_SPEED]*motor->polar;
	motor->statues.duty=usMRegHoldBuf[motor->id-1][SM_OUTPUT_PWM]*0.1;
	motor->statues.voltage=usMRegHoldBuf[motor->id-1][SM_INPUT_VOLTAGE]*0.1;
	motor->statues.temp=usMRegHoldBuf[motor->id-1][SM_INTERNAL_TEMP];
	motor->statues.is_moving=usMRegHoldBuf[motor->id-1][SM_MOVING_FLAG];
	motor->statues.current=usMRegHoldBuf[motor->id-1][SM_CURRENT_LOAD]*6.5;
	Motor_FTGetAngle(motor);//pos转换为angle
	return error_code;
}
/**
* @brief 飞特舵机同步写参数
* @param motor:舵机句柄
	* @retval error_code:MODBUS错误码
*/
eMBMasterReqErrCode Motor_FTWriteSynchronous(FT_SM2924_HandleTypeDef * motor)
{
	eMBMasterReqErrCode error_code = MB_MRE_NO_ERR;
	uint16_t tx_buffer[5];
	tx_buffer[0]=(motor->control.pos*motor->polar+motor->pos_offset);
	tx_buffer[1]=motor->control.torque_enable;
	tx_buffer[2]=motor->control.accelerate;
	tx_buffer[3]=motor->control.speed;
	tx_buffer[4]=motor->control.torque_limit;
	
	//eMBMasterReqWriteHoldingRegister(motor->id,SM_TARGET_POSITION,)
	error_code=eMBMasterReqWriteMultipleHoldingRegister(motor->id,SM_TARGET_POSITION,5,tx_buffer,100);
	return error_code;
}

