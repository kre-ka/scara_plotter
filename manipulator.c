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

bool calc_movement_time(float t_phases_out[3], float p, float v_0, float v_n, float *v_n_corrected){
	float t_acc, t_const_v, t_dcc;  // acceleration time, constant speed time, decceleration time
	float p_acc, p_const_v, p_dcc;  // similar, but for path

	t_acc = (V_TARGET - v_0) / ACC_MAX;
	t_dcc = (V_TARGET - v_n) / ACC_MAX;
	
	p_acc = (ACC_MAX * powf(t_acc, 2) / 2) + (v_0 * t_acc);
	p_dcc = (V_TARGET * t_dcc) - (ACC_MAX * powf(t_dcc, 2) / 2);
	p_const_v = p - p_acc - p_dcc;

	t_const_v = p_const_v / V_TARGET;

	bool success = true;
	// if didn't manage to reach target speed
	if (p_const_v < 0.0) {
		t_acc = (sqrtf(4*ACC_MAX*p + 3*powf(v_0, 2) - 4*v_0*v_n + 2*powf(v_n, 2)) - v_0) / (2*ACC_MAX);
		t_dcc = (v_0 - v_n) / ACC_MAX + t_acc;
		t_const_v = 0;
		// if didn't manage to reach v_n
		if (t_dcc < 0.0) {
			t_acc = (sqrtf(2*ACC_MAX*p + powf(v_0,2)) - v_0) / ACC_MAX;
			t_dcc = 0;
			// give corrected v_n value
			*v_n_corrected = ACC_MAX * t_acc;
			success = false;
		}
	}

	t_phases_out[0] = t_acc;
	t_phases_out[1] = t_acc + t_const_v;
	t_phases_out[2] = t_acc + t_const_v + t_dcc;

	return success;
}

void calc_p_trajectory(float *p_tab_out, float t_phases[3], float t_res, float p, float v_0){
	int idx_phases[3];
	idx_phases[0] = (int) roundf(t_phases[0] / t_res);
	idx_phases[1] = (int) roundf(t_phases[1] / t_res);
	idx_phases[2] = (int) floorf(t_phases[2] / t_res);
	
	// accceleration
	for(int i=0; i <= idx_phases[0]; i++) {
		float t = t_res * i;
		p_tab_out[i] = v_0 * t + ACC_MAX * powf(t, 2) / 2;
	}
	// in case of not reaching target speed
	// float v = ACC_MAX * t_res * idx_phases[0];
	float v = ACC_MAX * t_phases[1];
	// constant speed
	for(int i=idx_phases[0]+1; i <= idx_phases[1]; i++) {
		float t = t_res * (i - idx_phases[0]);
		p_tab_out[i] = p_tab_out[idx_phases[0]] + v * t;
	}
	// decceleration
	for(int i=idx_phases[1]+1; i <= idx_phases[2]; i++) {
		float t = t_res * (i - idx_phases[1]);
		p_tab_out[i] = p_tab_out[idx_phases[1]] + v * t - ACC_MAX * powf(t, 2) / 2;
		// just to be sure it doesn't go out of range
		if (p_tab_out[i] > p) p_tab_out[i] = p;
	}
}