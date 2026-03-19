#include "IR_Module.h"
// 引脚状态变量
uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;
//转向参数调参时放大100倍
float Turn90Angle  = 2.3f; // 直角弯转向角度（rad/s）
float maxTurnAngle = 1.03f; // 弯道最大转向角度（rad/s）
float midTurnAngle = 0.50f; // 弯道中等转向角度（rad/s）
float minTurnAngle = 0.25f; // 弯道最小转向角度（rad/s）
//调参时速度参数放大1000倍

void IRDM_line_inspection(void) {
    static int last_state = 0; // 上一控制周期的状态
    static int ten_time = 0;
	float baseSpeed=RC_Velocity/1000.0f;      //巡线基础速度:速度过快弯道易出轨-可根据实际调整
	    // 读取四个引脚的状态并强制转换为0或1
//    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;
//    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
//    ir_dh2_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
//    ir_dh1_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;
	    ir_dh4_state =0;
    ir_dh3_state = 0;
    ir_dh2_state =0;
    ir_dh1_state = 0;
	
    int DH_state = (ir_dh1_state << 3) | (ir_dh2_state << 2) | (ir_dh3_state << 1) | ir_dh4_state; // 将传感器状态组合成一个整数
    switch (DH_state) {
        case 0: // 0000 十字路口 - 停止两秒且鸣笛后直线通过
            ten_time++;
            if (ten_time < 1000) {
                Move_X = 0;
                Move_Z = 0;
            } else if (ten_time >= 1000) {
                Move_X = baseSpeed;
                Move_Z = 0;
            }
            last_state = 1;
            break;

        case 1: // 0001 左直角弯
            ten_time = 0;
            Move_X = baseSpeed * 0.3f;
            Move_Z = Turn90Angle;
            last_state = 2;
            break;

        case 8: // 1000 右直角弯
            ten_time = 0;
            Move_X = baseSpeed * 0.3f;
            Move_Z = -Turn90Angle;
            last_state = 3;
            break;

        case 7: // 0111 左大弯
            ten_time = 0;
            Move_X = baseSpeed * 0.7f;
            Move_Z = maxTurnAngle;
            last_state = 4;
            break;

        case 14: // 1110 右大弯
            ten_time = 0;
            Move_X = baseSpeed * 0.7f;
            Move_Z = -maxTurnAngle;
            last_state = 5;
            break;

        case 11: // 1011 左微调
            ten_time = 0;
            Move_X = baseSpeed * 0.8f;
            Move_Z = minTurnAngle;
            last_state = 6;
            break;

        case 13: // 1101 右微调
            ten_time = 0;
            Move_X = baseSpeed * 0.8f;
            Move_Z = -minTurnAngle;
            last_state = 7;
            break;

        case 9: // 1001 直行
            ten_time = 0;
            Move_X = baseSpeed;
            Move_Z = 0;
            last_state = 8;
            break;

       case 15://1111 丢线情况
            ten_time = 0;
			if(last_state==4||last_state==6){
				Move_X = baseSpeed * 0.8f;
				Move_Z = midTurnAngle;
			}
			if(last_state==5||last_state==7){
				Move_X = baseSpeed * 0.8f;
				Move_Z = -midTurnAngle;
			}
            break;
    }
}








