#include "math.h"
#include "ik.h"
#include "config.h"

const float PI=3.14159265359;

//从笛卡尔系坐标获取极坐标
// Get polar coords from cartesian ones
void cart2polar(float a, float b, float *r, float *theta)
{
    // Determine magnitude of cartesian coords
    *r = sqrt(a*a + b*b);

    // Don't try to calculate zero-magnitude vectors' angles
    if(r == 0) return;

    float c = a / *r;
    float s = b / *r;

    // Safety!
    if(s > 1) s = 1;
    if(c > 1) c = 1;
    if(s < -1) s = -1;
    if(c < -1) c = -1;

    // Calculate angle in 0..PI
    *theta = acos(c);

    // Convert to full range
    if(s < 0) *theta *= -1;
}
//使用余弦定理求三角形中的角度
// Get angle from a triangle using cosine rule
//返回0：失败，1：成功
uint8_t cosangle(float opp, float adj1, float adj2, float *theta)
{
    // Cosine rule:
    // C^2 = A^2 + B^2 - 2*A*B*cos(angle_AB)
    // cos(angle_AB) = (A^2 + B^2 - C^2)/(2*A*B)
    // C is opposite
    // A, B are adjacent
    float den = 2*adj1*adj2;

    if(den==0) return 0;
    float c = (adj1*adj1 + adj2*adj2 - opp*opp)/den;

    if(c>1 || c<-1) return 0;

    *theta = acos(c);

    return 1;
}

//解算角度
// Solve angles!
//返回值0：失败 1：成功
uint8_t solve(float x, float y, float z, float *a0, float *a1, float *a2)
{
    // Solve top-down view
    float r, th0;
    cart2polar(y, x, &r, &th0);

    // Account for the wrist length!
    r -= L3;

    // In arm plane, convert to polar
    float ang_P, R;
    cart2polar(r, z, &R, &ang_P);

    // Solve arm inner angles as required
    float B, C;
    if(!cosangle(L2, L1, R, &B)) return 0;
    if(!cosangle(R, L1, L2, &C)) return 0;

    // Solve for servo angles from horizontal
    *a0 = th0;
    *a1 = ang_P + B;
    *a2 = C + *a1 - PI;

    return 1;
}