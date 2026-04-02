#ifndef __CALCULATION_H
#define __CALCULATION_H

#include <rtthread.h>
#include <rtdevice.h>

void forward_kinematics(double theta1, double theta2, double *x, double *y);
int inverse_kinematics(double x, double y, double *theta1, double *theta2);
uint8_t get_better_inverse(double *theta1_1, double *theta2_1, double *theta1_2, double *theta2_2);
#endif