


#include "chassis_module_mai.h"
#include "../drv_emm_v5.h"
#include "math.h"
#include "stdlib.h"
#include "ulog_def.h"
#include <stdint.h>

#define DBG_TAG "Chassis.mai"
#define DBG_LVL DBG_DBG
#include <ulog.h>

#ifdef CHASSIS_MODULE_MAI
int module_mai(struct chassis *chassis, const void *output, const void *input, chassis_status require_cmd);
#ifdef CHASSIS_USING_MOTOR_HAL
static int driver_mai(struct chassis *chassis, const void *output, const void *input, chassis_status require_cmd);
#endif

chassis_mai_data_t mai_data;

chassis_ops_t ops_mai =
{
  .module = module_mai,
#ifdef CHASSIS_USING_MOTOR_HAL
  .driver = driver_mai,
#else
  .driver = NULL,
#endif
  .date = &mai_data
};

//#define CHASSIS_MAI_WHELL_R_M (0.036804f) // 3个都差1mm 导致地盘旋转180会有5.6度偏差，，，测量误差！！！！！没想过这些会影响这么大，，学会使用工具和是适当的替换距离
#define CHASSIS_MAI_WHELL_R_M (0.03735f) // 3个都差1mm 导致地盘旋转180会有5.6度偏差，，，测量误差！！！！！没想过这些会影响这么大，，学会使用工具和是适当的替换距离
//#define CHASSIS_MAI_WHELL_R_M (0.034904f) // 3个都差1mm 导致地盘旋转180会有5.6度偏差，，，测量误差！！！！！没想过这些会影响这么大，，学会使用工具和是适当的替换距离
#define CHSSIS_MAI_A_M (0.22325f)//0.185
#define CHSSIS_MAI_B_M (0.185f)//0.215


#define PI (3.14159265359f)
#define DEG_TO_RAD 1
#define CHASSIS_HALF_A_B (0.5 * (CHSSIS_MAI_A_M + CHSSIS_MAI_B_M))
#define CHASSIS_A_B ((CHSSIS_MAI_A_M + CHSSIS_MAI_B_M))
#define COS45 (0.70710678118)
#define SIN45 (0.70710678118)
#define CHASSIS_2PIR (6.28318530718 * CHASSIS_MAI_WHELL_R_M)
#define CHASSIS_R (sqrt((CHSSIS_MAI_A_M / 2.f) * (CHSSIS_MAI_A_M / 2.f) + (CHSSIS_MAI_B_M / 2.f) * (CHSSIS_MAI_B_M / 2.f)))

#define conversion (180.f / CHASSIS_MAI_WHELL_R_M / PI)
int module_mai(struct chassis *chassis, const void *output, const void *input, chassis_status require_cmd)
{
  if (output != NULL)
    {
      chassis_mai_data_t *data = (chassis_mai_data_t *)output;
      data->type = require_cmd;
      switch (require_cmd)
        {
        case CHASSIS_SPEED:
          // 速度控制
          //保存上次
//            data->motor1 = (( (chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) + (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) - ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B)) / CHASSIS_2PIR * 60.f;
//            data->motor2 = (( (chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) - (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) - ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B)) / CHASSIS_2PIR * 60.f;
//            data->motor3 = ((-(chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) - (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) - ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B)) / CHASSIS_2PIR * 60.f;
//            data->motor4 = ((-(chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) + (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) - ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B)) / CHASSIS_2PIR * 60.f;

          data->motor1 = (( (chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) + (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) - ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B*DEG_TO_RAD)) / CHASSIS_2PIR * 60.f;
          data->motor2 = ((-(chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) + (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) + ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B*DEG_TO_RAD)) / CHASSIS_2PIR * 60.f;
          data->motor3 = ((-(chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) + (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) - ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B*DEG_TO_RAD)) / CHASSIS_2PIR * 60.f;
          data->motor4 = (((chassis->target.speed.x_m_s + chassis->offset.speed.x_m_s) + (chassis->target.speed.y_m_s + chassis->offset.speed.y_m_s)) + ((chassis->target.speed.z_rad_s+chassis->offset.speed.z_rad_s) * CHASSIS_HALF_A_B*DEG_TO_RAD)) / CHASSIS_2PIR * 60.f;
          break;
        case CHASSIS_POS:
          // 位置控制
//            data->motor1 = (( (chassis->target.pos.x_m + chassis->offset.pos.x_m) +  (chassis->target.pos.y_m + chassis->offset.pos.y_m)) - ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B)) * conversion;  // 430  chassis->target.pos.z_rad  CHASSIS_R
//            data->motor2 = (( (chassis->target.pos.x_m + chassis->offset.pos.x_m) -  (chassis->target.pos.y_m + chassis->offset.pos.y_m)) - ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B)) * conversion;  //-430
//            data->motor3 = ((-(chassis->target.pos.x_m + chassis->offset.pos.x_m) - ( chassis->target.pos.y_m + chassis->offset.pos.y_m)) - ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B)) * conversion; //-430          1433
//            data->motor4 = ((-(chassis->target.pos.x_m + chassis->offset.pos.x_m) + ( chassis->target.pos.y_m + chassis->offset.pos.y_m)) - ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B)) * conversion; // 430
          data->motor1 = (( (chassis->target.pos.x_m + chassis->offset.pos.x_m) +  (chassis->target.pos.y_m + chassis->offset.pos.y_m)) - ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B*DEG_TO_RAD)) * conversion;  // 430  chassis->target.pos.z_rad  CHASSIS_R
          data->motor2 = ((-(chassis->target.pos.x_m + chassis->offset.pos.x_m) +  (chassis->target.pos.y_m + chassis->offset.pos.y_m)) + ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B*DEG_TO_RAD)) * conversion;  //-430
          data->motor3 = ((-(chassis->target.pos.x_m + chassis->offset.pos.x_m) + ( chassis->target.pos.y_m + chassis->offset.pos.y_m)) - ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B*DEG_TO_RAD)) * conversion; //-430          1433
          data->motor4 = (((chassis->target.pos.x_m + chassis->offset.pos.x_m) + ( chassis->target.pos.y_m + chassis->offset.pos.y_m)) + ((chassis->target.pos.z_rad +chassis->offset.pos.z_rad) * CHASSIS_HALF_A_B*DEG_TO_RAD)) * conversion; // 430
          break;
        default:
          break;
        }
    }
  if (input != NULL)
    {
      chassis_mai_data_t *data = (chassis_mai_data_t *)input;
      switch (require_cmd)
        {
        case CHASSIS_SPEED:
          // 速度控制
          chassis->present.speed.x_m_s = chassis->offset.speed.x_m_s + ((data->motor1 - data->motor2) / CHASSIS_2PIR * 60.f) / 2.f;
          chassis->present.speed.y_m_s = chassis->offset.speed.y_m_s + ((-data->motor2 + data->motor4) / CHASSIS_2PIR * 60.f) / 2.f;
          chassis->present.speed.z_rad_s = chassis->offset.speed.z_rad_s + (-(data->motor3 + data->motor2) / CHASSIS_2PIR * 60.f) / (2.f * (CHASSIS_HALF_A_B));
          break;
        case CHASSIS_POS:

          // 位置控制
          // chassis->present.pos.x_m = ((data->motor1 + data->motor2) /  ( conversion)) / 2.f;
          // chassis->present.pos.y_m = ((-data->motor2 + data->motor4 ) /( conversion)) / 2.f;
          // chassis->present.pos.z_rad = (-(-data->motor3 + data->motor2) /( conversion)) / (2.f * CHASSIS_HALF_A_B);
          chassis->present.pos.x_m = chassis->offset.pos.x_m + (data->motor1 + data->motor2 - data->motor3 - data->motor4) / 4.f / conversion;
          chassis->present.pos.y_m = chassis->offset.pos.y_m + (data->motor1 - data->motor2 - data->motor3 + data->motor4) / 4.f / conversion;
          chassis->present.pos.z_rad = chassis->offset.pos.z_rad - (data->motor1 + data->motor2 + data->motor3 + data->motor4) / CHASSIS_HALF_A_B / 4.f / conversion;
          //LOG_D("xm:%f,ym:%f,zrad:%f", chassis->present.pos.x_m, chassis->present.pos.y_m, chassis->present.pos.z_rad);

          break;
        default:

          break;
        }
      return 0;
    }
  return 0;
}
#ifdef CHASSIS_USING_MOTOR_HAL
extern stepper_motor_t stepper_motor_1;
extern stepper_motor_t stepper_motor_2;
extern stepper_motor_t stepper_motor_3;
extern stepper_motor_t stepper_motor_4;

uint8_t send_start = 1;
static int driver_mai(struct chassis *chassis, const void *output, const void *input, chassis_status require_cmd)
{
  if (input != NULL)
    {
      // 读取电机数据输出
      chassis_mai_data_t *data = (chassis_mai_data_t *)input;
      switch (require_cmd)
        {
        case CHASSIS_SPEED:
          // 速度控制
          // LOG_D("speed get motor1:%f motor2:%f motor3:%f motor4:%f\n", data->motor1, data->motor2, data->motor3, data->motor4);
          data->motor1 = stepper_motor_1.stepper_motor_speed;
          data->motor2 = stepper_motor_2.stepper_motor_speed;
          data->motor3 = stepper_motor_3.stepper_motor_speed;
          data->motor4 = stepper_motor_4.stepper_motor_speed;
          break;
        case CHASSIS_POS:
          // 位置控制
          // LOG_D("pos get motor1:%f motor2:%f motor3:%f motor4:%f\n", data->motor1, data->motor2, data->motor3, data->motor4);
          data->motor1 = stepper_motor_1.stepper_motor_angle;
          data->motor2 = stepper_motor_2.stepper_motor_angle;
          data->motor3 = stepper_motor_3.stepper_motor_angle;
          data->motor4 = stepper_motor_4.stepper_motor_angle;
          break;
        default:
          break;
        }
    }
  if (output != NULL)
    {
      // 写入电机数据
      chassis_mai_data_t *data = (chassis_mai_data_t *)output;
      switch (require_cmd)
        {
        case CHASSIS_SPEED:
          // 速度控制
          LOG_D("speed set motor1:%f motor2:%f motor3:%f motor4:%f\n", data->motor1, data->motor2, data->motor3, data->motor4);
          data->motor1 = -data->motor1;
          data->motor2 = data->motor2;
          data->motor3 = -data->motor3;
          data->motor4 = data->motor4;
				
          Emm_V5_Vel_Control(&stepper_motor_1,data->motor1>0?1:0,data->motor1>0?(uint16_t)(data->motor1):(uint16_t)(-1*data->motor1),50,0);
          Emm_V5_Vel_Control(&stepper_motor_2,data->motor2>0?1:0,data->motor2>0?(uint16_t)(data->motor2):(uint16_t)(-1*data->motor2),50,0);
          Emm_V5_Vel_Control(&stepper_motor_3,data->motor3>0?1:0,data->motor3>0?(uint16_t)(data->motor3):(uint16_t)(-1*data->motor3),50,0);
          Emm_V5_Vel_Control(&stepper_motor_4,data->motor4>0?1:0,data->motor4>0?(uint16_t)(data->motor4):(uint16_t)(-1*data->motor4),50,0);
//
//            motor_set_speed(MOTOR_MAI_ID_1, data->motor1);
//            motor_set_speed(MOTOR_MAI_ID_2, data->motor2);
//            motor_set_speed(MOTOR_MAI_ID_3, data->motor3);
//            motor_set_speed(MOTOR_MAI_ID_4, data->motor4);
          break;
        case CHASSIS_POS:
          // 位置控制

//            motor_set_pos(MOTOR_MAI_ID_1, data->motor1);
//            motor_set_pos(MOTOR_MAI_ID_2, data->motor2);
//            motor_set_pos(MOTOR_MAI_ID_3, data->motor3);
//            motor_set_pos(MOTOR_MAI_ID_4, data->motor4);
          data->motor1 = -data->motor1;
          data->motor2 = data->motor2;
          data->motor3 = -data->motor3;
          data->motor4 = data->motor4;
          //将结算出来的目标值赋值到电机结构体的目标角度中，方便在外部读取
          stepper_motor_1.stepper_motor_target_angle = data->motor1;
          stepper_motor_2.stepper_motor_target_angle = data->motor2;
          stepper_motor_3.stepper_motor_target_angle = data->motor3;
          stepper_motor_4.stepper_motor_target_angle = data->motor4;
          //          LOG_D("pos set motor1:%f motor2:%f motor3:%f motor4:%f\n",
          //          data->motor1, data->motor2, data->motor3, data->motor4);
          if(send_start == 1)
            {
                      Emm_V5_Pos_Control(&stepper_motor_1,data->motor1>0?1:0,200,156,fabsf(data->motor1*6400/360.0f),true,true);
                      Emm_V5_Pos_Control(&stepper_motor_2,data->motor2>0?1:0,200,156,fabsf(data->motor2*6400/360.0f),true,true);
                      Emm_V5_Pos_Control(&stepper_motor_3,data->motor3>0?1:0,200,156,fabsf(data->motor3*6400/360.0f),true,true);
                      Emm_V5_Pos_Control(&stepper_motor_4,data->motor4>0?1:0,200,156,fabsf(data->motor4*6400/360.0f),true,true);
                      Emm_V5_Synchronous_motion(&stepper_motor_1);
                      LOG_D("pos set motor1:%f motor2:%f motor3:%f motor4:%f\n", data->motor1, data->motor2, data->motor3, data->motor4);
                      send_start = 0;
          }

          // if(fabs(data->motor1 + stepper_motor_1.stepper_motor_angle) < 0.5f && fabs(data->motor2 + stepper_motor_2.stepper_motor_angle) < 0.5f &&\
          //     fabs(data->motor3 + stepper_motor_3.stepper_motor_angle) < 0.5f && fabs(data->motor4 + stepper_motor_4.stepper_motor_angle) < 0.5f && state_start == 0)
          //   {
          //   Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_1);
          //   Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_2);
          //   Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_3);
          //   Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_4);
          //   state_start = 1;
          //   send_start = 1;
          //   }

          break;
        default:
          break;
        }
    }
  return 0;
}

uint8_t motor_moveOK(void) {
  if (fabs(stepper_motor_1.stepper_motor_target_angle +
           stepper_motor_1.stepper_motor_angle) < 1.0f &&
      fabs(stepper_motor_2.stepper_motor_target_angle +
           stepper_motor_2.stepper_motor_angle) < 1.0f &&
      fabs(stepper_motor_3.stepper_motor_target_angle +
           stepper_motor_3.stepper_motor_angle) < 1.0f &&
      fabs(stepper_motor_4.stepper_motor_target_angle +
           stepper_motor_4.stepper_motor_angle) < 1.0f ) {
    Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_1);
    Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_2);
    Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_3);
    Emm_V5_Reset_CurPos_To_Zero(&stepper_motor_4);
		
						 
		stepper_motor_1.stepper_motor_target_angle=0;
		stepper_motor_2.stepper_motor_target_angle=0;
		stepper_motor_3.stepper_motor_target_angle=0;
		stepper_motor_4.stepper_motor_target_angle=0;
						 
		stepper_motor_1.stepper_motor_angle=0;
		stepper_motor_2.stepper_motor_angle=0;
		stepper_motor_3.stepper_motor_angle=0;
		stepper_motor_4.stepper_motor_angle=0;
						 
						 
//    send_start = 1;
    return 1;
  }
  else return 0;
}

#endif
#endif
