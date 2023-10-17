#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "manipulator.h"


#define RAD_TO_DEG 180/M_PI

int main(){
    Manipulator manipulator;
    manipulator.l_0 = 100.0;
    manipulator.l_1 = 50.0;
    manipulator.theta_0_range[0] = -M_PI_2;
    manipulator.theta_0_range[1] = M_PI_2;
    manipulator.theta_1_range[0] = -M_PI_2;
    manipulator.theta_1_range[1] = M_PI_2;

    float in[] = {100.0, 50.0};
    float out[2];
    int mode = -1;
    inverse_kinematics(&manipulator, &in[0], &out[0], mode);
    printf("in: %f, %f\n", in[0], in[1]);
    printf("out: %f, %f\n", out[0]*RAD_TO_DEG, out[1]*RAD_TO_DEG);
    
    return 0;
}