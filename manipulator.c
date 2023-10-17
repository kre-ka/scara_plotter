#include "manipulator.h"
#include <math.h>

bool is_in_range_angle(Manipulator *manipulator, float *theta){
	return theta[0] >= manipulator->theta_0_range[0] && theta[0] <= manipulator->theta_0_range[1] &&
		   theta[1] >= manipulator->theta_1_range[0] && theta[1] <= manipulator->theta_1_range[1];
}

int inverse_kinematics(Manipulator *manipulator, float *in, float *out, int mode){
	float r = sqrtf(powf(in[0], 2) + powf(in[1], 2));
	out[0] = atan2f(in[1], in[0]) - mode * acosf((powf(manipulator->l_0, 2) - powf(manipulator->l_1, 2) + powf(r, 2)) /
			(2 * manipulator->l_0 * r));
	out[1] = mode * acosf((powf(r, 2) - powf(manipulator->l_0, 2) - powf(manipulator->l_1, 2)) /
			(2 * manipulator->l_0 * manipulator->l_1));
	return 0;
}

