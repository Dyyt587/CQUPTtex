#ifndef MEARM_H
#define MEARM_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stdbool.h>
//const float pi=3.14159265359;

struct ServoInfo {
    int n_min, n_max;   // PWM 'soft' limits - should be just within range
    float gain;         // PWM per radian
    float zero;         // Theoretical PWM for zero angle
};

void meArm_Init(void);
void meArm_goDirectlyTo(float x, float y, float z);
void meArm_gotoPoint(float x, float y, float z);
void meArm_polarToCartesian(float theta, float r, float *x, float  *y);
void meArm_gotoPointCylinder(float theta, float r, float z);
void meArm_goDirectlyToCylinder(float theta, float r, float z);
uint8_t meArm_isReachable(float x, float y, float z);
void meArm_openGripper();
void meArm_closeGripper();


void car_run(float x,float y,float z);
void meArm_Gripper_Get(void);
void meArm_Gripper_Release_1(void);
void meArm_Gripper_Release_2(void);
void meArm_Gripper_Release_3_1(void);
void meArm_Gripper_Release_3_2(void);

#endif