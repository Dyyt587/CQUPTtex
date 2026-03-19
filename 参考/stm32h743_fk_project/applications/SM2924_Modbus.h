#ifndef __SM2924_MODBUS_H
#define __SM2924_MODBUS_H

#include <rtthread.h>
#include <rtdevice.h>
#include "user_mb_app.h"



//extern USHORT   usMRegInStart                              ;
//extern USHORT   usMRegInBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_INPUT_NREGS];
////Master mode:HoldingRegister variables
//extern USHORT   usMRegHoldStart                            ;
//extern USHORT   usMRegHoldBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];

/* EPROM Area */
#define SM_FIRMWARE_VERSION         0x0000  // 40001: Firmware Version
#define SM_SERVO_VERSION            0x0001  // 40002: Servo Version
#define SM_FIRMWARE_YEAR            0x0002  // 40003: Firmware Release Year
#define SM_FIRMWARE_MONTH           0x0003  // 40004: Firmware Release Month

#define SM_DEVICE_ID                0x000A  // 40011: Bus ID
#define SM_BAUDRATE                 0x000B  // 40012: Baudrate
#define SM_RETURN_DELAY             0x000C  // 40013: Response Delay
#define SM_MIN_ANGLE_LIMIT          0x000D  // 40014: Min Angle Limit
#define SM_MAX_ANGLE_LIMIT          0x000E  // 40015: Max Angle Limit
#define SM_POSITION_OFFSET          0x000F  // 40016: Position Calibration
#define SM_OPERATION_MODE           0x0010  // 40017: Operation Mode
#define SM_POSITION_P_GAIN          0x0011  // 40018: Position P Gain
#define SM_POSITION_D_GAIN          0x0012  // 40019: Position D Gain
#define SM_POSITION_I_GAIN          0x0013  // 40020: Position I Gain
#define SM_VELOCITY_P_GAIN          0x0014  // 40021: Velocity P Gain
#define SM_VELOCITY_I_GAIN          0x0015  // 40022: Velocity I Gain

/* SRAM Area */
#define SM_TARGET_POSITION          0x0080  // 40129: Target Position
#define SM_TORQUE_ENABLE            0x0081  // 40130: Torque Switch
#define SM_ACCELERATION             0x0082  // 40131: Acceleration
#define SM_MOVING_SPEED             0x0083  // 40132: Moving Speed
#define SM_TORQUE_LIMIT             0x0084  // 40133: Torque Limit
#define SM_EEPROM_LOCK              0x0085  // 40134: Write Lock Flag
#define SM_FAULT_RESET              0x0086  // 40135: Fault Reset

/* Read-Only Area */
#define SM_SERVO_STATUS             0x0100  // 40257: Servo Status
#define SM_CURRENT_POSITION         0x0101  // 40258: Current Position
#define SM_CURRENT_SPEED            0x0102  // 40259: Current Speed
#define SM_OUTPUT_PWM               0x0103  // 40260: Output PWM
#define SM_INPUT_VOLTAGE            0x0104  // 40261: Input Voltage
#define SM_INTERNAL_TEMP            0x0105  // 40262: Internal Temperature
#define SM_MOVING_FLAG              0x0106  // 40263: Moving Status
#define SM_CURRENT_LOAD             0x0107  // 40264: Current Load

/* Default Parameters Area */
#define SM_MOVE_THRESHOLD           0x0180  // 40385: Movement Detection Threshold
#define SM_D_CONTROL_TIME           0x0181  // 40386: D Control Time
#define SM_MAX_SPEED_LIMIT          0x0182  // 40387: Max Speed Limit
#define SM_MIN_SPEED_LIMIT          0x0183  // 40388: Min Speed Limit
#define SM_ACCEL_LIMIT              0x0184  // 40389: Acceleration Limit
#define SM_STARTUP_TORQUE           0x0185  // 40390: Startup Torque
#define SM_CW_DEADZONE              0x0186  // 40391: CW Deadzone
#define SM_CCW_DEADZONE             0x0187  // 40392: CCW Deadzone
#define SM_PHASE_SETTING            0x0188  // 40393: Phase Setting
#define SM_PROTECTION_SWITCH        0x0189  // 40394: Protection Switch
#define SM_LED_ALARM_CONDITION      0x018A  // 40395: LED Alarm Condition
#define SM_TEMP_LIMIT               0x018B  // 40396: Temperature Limit
#define SM_VOLTAGE_HIGH_LIMIT       0x018C  // 40397: High Voltage Limit
#define SM_VOLTAGE_LOW_LIMIT        0x018D  // 40398: Low Voltage Limit
#define SM_OVERLOAD_CURRENT         0x018E  // 40399: Overload Current
#define SM_OVERCURRENT_TIME         0x018F  // 40400: Overcurrent Protection Time
#define SM_SAFETY_TORQUE            0x0190  // 40401: Protection Torque
#define SM_OVERLOAD_TORQUE          0x0191  // 40402: Overload Torque Threshold
#define SM_OVERLOAD_PROTECT_TIME    0x0192  // 40403: Overload Protection Time
#define SM_ANGLE_RESOLUTION         0x0193  // 40404: Angle Resolution
#define SM_TORQUE_DEFAULT           0x0194  // 40405: Default Torque Limit
#define SM_ACCEL_DEFAULT            0x0195  // 40406: Default Acceleration
#define SM_SPEED_DEFAULT            0x0196  // 40407: Default Speed



#define speed_default 50            // 速度默认值
#define accel_default 100           // 加速度默认值
#define torque_limit_default 1000    // 扭矩限制默认值

#define ft_pi 3.14f	 //数字pi

typedef struct {
    int16_t pos;                     // 目标位置（-32768-32767，舵机模式下为0-4095，对应0-360度）
    uint8_t torque_enable;           // 扭矩使能开关（0关闭,1开启）
    uint16_t accelerate;             // 加速度值（0-65535）
    uint16_t speed;                  // 速度值（0-65535）
    uint16_t torque_limit;           // 扭矩限制值（0-1000）
} Motor_ParameterTypeDef;            // 电机参数句柄

typedef struct {
    uint16_t servo_status;           // 舵机状态
    int16_t  pos;                    // 当前位置（返回值为有符号整数）
		float angle;										 //当前角度值（由当前位置计算得出）
    int16_t speed;                   // 当前速度值
    float duty;                      // PWM占空比（0-100%）
    float voltage;                   // 当前电压（单位：V）
    uint16_t temp;                   // 温度值（单位：℃）
    uint8_t is_moving;               // 是否移动到位（0-已到位，1-运动中）
    uint16_t current;                // 电流值（0-2470mA）
} Motor_StatusTypeDef;               // 反馈参数句柄

typedef struct{
	uint8_t  id;													//电机id		0-127
	int8_t  polar;												//电机极性
	uint16_t pos_offset;									//位置偏移
	Motor_ParameterTypeDef control;				//控制参数
	Motor_StatusTypeDef statues;					//反馈参数
}FT_SM2924_HandleTypeDef;

//extern FT_SM2924_HandleTypeDef Motor_FT2;
//extern FT_SM2924_HandleTypeDef Motor_FT3;
//extern FT_SM2924_HandleTypeDef Motor_FT4;
extern FT_SM2924_HandleTypeDef Motor_FT_Left;
extern FT_SM2924_HandleTypeDef Motor_FT_Right;

void Motor_FTInit(FT_SM2924_HandleTypeDef * motor,uint8_t id);
void Motor_FTSetPosOffSet(FT_SM2924_HandleTypeDef * motor,uint16_t offset);
void Motor_FTSetPos(FT_SM2924_HandleTypeDef * motor,int16_t pos);
void Motor_FTSetAngle(FT_SM2924_HandleTypeDef * motor,float angle);
void Motor_FTSetPolar(FT_SM2924_HandleTypeDef * motor,int8_t polar);
eMBMasterReqErrCode Motor_FTReadSynchronous(FT_SM2924_HandleTypeDef * motor);
eMBMasterReqErrCode Motor_FTReadValid(FT_SM2924_HandleTypeDef * motor);
eMBMasterReqErrCode Motor_FTWriteSynchronous(FT_SM2924_HandleTypeDef * motor);

#endif
