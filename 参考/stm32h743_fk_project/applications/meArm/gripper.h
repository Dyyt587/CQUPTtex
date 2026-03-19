#ifndef GRIPPER_H
#define GRIPPER_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stdbool.h>

void Gripper_Init(float angle);
void Gripper_Set_Angle(float angle);

#endif