#include "calculation.h"
#include <stdio.h>
#include <math.h>

// 定义机械臂的长度
#define L1 0.2 // 第一段臂长
#define L2 0.2 // 第二段臂长

#define motor1_low -0.34f
#define motor1_up 3.5f
#define motor2_low -2.6f
#define motor2_up 2.6f


// 正向运动学计算
void forward_kinematics(double theta1, double theta2, double *x, double *y) {
    *x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    *y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
}

// 逆向运动学计算
int inverse_kinematics(double x, double y, double *theta1, double *theta2) {
    double c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (c2 < -1 - 1e-6 || c2 > 1 + 1e-6) {
        // 超出机械臂的工作空间
        return -1;
    }
    c2 = fmax(-1, fmin(1, c2)); // 修正c2到合法范围内
		double theta2_1=0,theta2_2=0,theta1_1=0,theta1_2=0;
    // 正解
    double s2_positive = sqrt(1 - c2 * c2);
    theta2_1 = atan2(s2_positive, c2);

    double k1_positive = L1 + L2 * c2;
    double k2_positive = L2 * s2_positive;
    theta1_1 = atan2(y, x) - atan2(k2_positive, k1_positive);

    // 负解
    double s2_negative = -sqrt(1 - c2 * c2);
    theta2_2 = atan2(s2_negative, c2);

    double k1_negative = L1 + L2 * c2;
    double k2_negative = L2 * s2_negative;
    theta1_2 = atan2(y, x) - atan2(k2_negative, k1_negative);
		//寻找最优解
		uint8_t retval=get_better_inverse(&theta1_1,&theta2_1,&theta1_2,&theta2_2);
		if(retval==0)
		{
			return 0;
		}
		else if(retval==1)
		{
			*theta1=theta1_1;
			*theta2=theta2_1;
			return 1;
		}
		else if(retval==2)
		{
			*theta1=theta1_2;
			*theta2=theta2_2;
			return 2;
		}
    return 0;
}

uint8_t get_better_inverse(double *theta1_1, double *theta2_1, double *theta1_2, double *theta2_2)
{
	if(*theta1_1>=motor1_low&&*theta1_1<=motor1_up\
		&&*theta2_1>=motor2_low&&*theta2_1<=motor2_up)
	{
		return 1;
	}
	if(*theta1_2>=motor1_low&&*theta1_2<=motor1_up\
		&&*theta2_2>=motor2_low&&*theta2_2<=motor2_up)
	{
		return 2;
	}
	return 0;
}


