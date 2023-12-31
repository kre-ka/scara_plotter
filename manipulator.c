#include "manipulator.h"
#include "stdio.h"
#define _USE_MATH_DEFINES
#include <math.h>

bool manipulator_init(Manipulator *manipulator, float l_0, float l_1, float theta_0_min, float theta_0_max, float theta_1_min, float theta_1_max){
	// validate input
	bool result = true;
	if (l_0 <= 0 || l_1 <= 0){
		result = false;
		printf("l_0 and l_1 must be greater than 0.\n");
	}
	if (theta_0_max < theta_0_min || theta_1_max < theta_1_min){
		result = false;
		printf("Maximal theta values must be greater than minial values.\n");
	}
	if (!result) return false;

	// set manipulator configuration based on theta_1 range
	// it will only work in one configuration (for simplicity)
	if (fabsf(theta_1_max) >= fabsf(theta_1_min)) manipulator->configuration = RIGHT;
	else manipulator->configuration = LEFT;
	
	manipulator->l_0 = l_0;
	manipulator->l_1 = l_1;
	manipulator->theta_0_min = theta_0_min;
	manipulator->theta_0_max = theta_0_max;
	manipulator->theta_1_min = theta_1_min;
	manipulator->theta_1_max = theta_1_max;
	
	// work area data
	// these are helper variables to avoid duplicating code for both configurations
	float theta_0_abs_min, theta_0_abs_max, theta_1_abs_min, theta_1_abs_max;
	if (manipulator->configuration == RIGHT) {
		theta_0_abs_min = theta_0_max;
		theta_0_abs_max = theta_0_min;
		theta_1_abs_min = theta_1_min;
		theta_1_abs_max = theta_1_max;
	}
	else {
		theta_0_abs_min = theta_0_min;
		theta_0_abs_max = theta_0_max;
		theta_1_abs_min = theta_1_max;
		theta_1_abs_max = theta_1_min;
	}
	// minimal x value (<=0)
	float x_min = l_0*cosf(theta_0_abs_max) + l_1*cosf(theta_0_abs_max + theta_1_abs_max);
	if (x_min < 0.0) x_min = 0.0;
	manipulator->work_area_data.x_min = x_min;
	// minimal range radius squared (center at (0,0))
	manipulator->work_area_data.r_min_sqr = powf(l_0, 2) + powf(l_1, 2) - 2*l_0*l_1*cosf(M_PI-theta_1_abs_max);
	// maximal range radius squared for theta_1=0 (manipulator fully straight; center at (0,0))
	manipulator->work_area_data.r_max_straight_sqr = powf(l_0 + l_1*cosf(theta_1_abs_min), 2);
	// maximal range radius squared for theta_0=max (moving theta_1 when theta_0 is at maximum position; center at (x_center_edge, y_center_edge))
	manipulator->work_area_data.r_max_edge_sqr = powf(l_1, 2);
	// center of maximal range circle for theta_0=max
	manipulator->work_area_data.x_center_edge = l_0*cosf(theta_0_abs_min);
	manipulator->work_area_data.y_center_edge = l_0*sinf(theta_0_abs_min);
	// border between r_max_straight and r_max_edge appliance area (based on last point where manipulator can be fully straight)
	manipulator->work_area_data.y_border = (l_0 + l_1)*sinf(theta_0_abs_min);

	return true;
}

void manipulator_print_work_area(Manipulator *manipulator){
	ManipulatorWorkAreaData *work_area = &(manipulator->work_area_data);
	printf("--- MANIPULATOR WORK AREA ---\n");
	printf("x >= %.2f\n", work_area->x_min);
	printf("x^2 + y^2 >= %.2f^2\n", sqrtf(work_area->r_min_sqr));
	if (manipulator->configuration == RIGHT) {
		printf("x^2 + y^2 <= %.2f^2 for y <= %.2f\n", sqrtf(work_area->r_max_straight_sqr), work_area->y_border);
		printf("(x - %.2f)^2 + (y - %.2f)^2 <= %.2f^2 for y > %.2f\n", work_area->x_center_edge, work_area->y_center_edge, sqrtf(work_area->r_max_edge_sqr), work_area->y_border);
	}
	else {
		printf("x^2 + y^2 <= %.2f^2 for y >= %.2f\n", sqrtf(work_area->r_max_straight_sqr), work_area->y_border);
		printf("(x - %.2f)^2 + (y - %.2f)^2 <= %.2f^2 for y < %.2f\n", work_area->x_center_edge, work_area->y_center_edge, sqrtf(work_area->r_max_edge_sqr), work_area->y_border);
	}
	printf("---\n");
}

bool is_in_range_angle(Manipulator *manipulator, float theta[2]){
	return theta[0] >= manipulator->theta_0_min && theta[0] <= manipulator->theta_0_max &&
		   theta[1] >= manipulator->theta_1_min && theta[1] <= manipulator->theta_1_max;
}

bool is_in_range_work_area(Manipulator *manipulator, float point[2]){
	// x >= x_min
	if (point[0] < manipulator->work_area_data.x_min) return false;
	float r_sqr = powf(point[0], 2) + powf(point[1], 2);
	// r_sqr >= r_min_sqr
	if (r_sqr < manipulator->work_area_data.r_min_sqr) return false;
	// (y <= y_border) XOR (config == RIGHT)
	// looks scary, but otherwise i would have to duplicate code
	if ((point[1] <= manipulator->work_area_data.y_border) != (manipulator->configuration == RIGHT)) {
		// r_sqr <= r_max_straight_sqr
		if (r_sqr > manipulator->work_area_data.r_max_straight_sqr) return false;
	}
	else {
		// r_sqr <= r_max_edge_sqr
		if (powf(point[0] - manipulator->work_area_data.x_center_edge, 2) + 
			powf(point[1] - manipulator->work_area_data.y_center_edge, 2) > 
			manipulator->work_area_data.r_max_edge_sqr) return false;
	}
	return true;
}

void inverse_kinematics(Manipulator *manipulator, float in[2], float out[2]){
	float r = sqrtf(powf(in[0], 2) + powf(in[1], 2));
	out[0] = atan2f(in[1], in[0]) - manipulator->configuration * acosf((powf(manipulator->l_0, 2) - powf(manipulator->l_1, 2) + powf(r, 2)) /
			(2 * manipulator->l_0 * r));
	out[1] = manipulator->configuration * acosf((powf(r, 2) - powf(manipulator->l_0, 2) - powf(manipulator->l_1, 2)) /
			(2 * manipulator->l_0 * manipulator->l_1));
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
	float v = ACC_MAX * t_phases[0];
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