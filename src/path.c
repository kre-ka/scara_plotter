#include "path.h"
#include <math.h>


bool path_init(Path *path, float p, float v_0, float *v_n_ptr, float v_target, float acc_max) {
    float v_n = *v_n_ptr;
    float t_acc, t_const_v, t_dcc;  // acceleration time, constant speed time, decceleration time
	float p_acc, p_const_v, p_dcc;  // similar, but for path

	t_acc = (v_target - v_0) / acc_max;
	t_dcc = (v_target - v_n) / acc_max;
	
	p_acc = (acc_max * powf(t_acc, 2) / 2) + (v_0 * t_acc);
	p_dcc = (v_target * t_dcc) - (acc_max * powf(t_dcc, 2) / 2);
	p_const_v = p - p_acc - p_dcc;

	t_const_v = p_const_v / v_target;

	float v_peak = v_target;
	bool success = true;
	// if didn't manage to reach target speed
	if (p_const_v < 0.0) {
		t_acc = (sqrtf(4*acc_max*p + 3*powf(v_0, 2) - 4*v_0*v_n + 2*powf(v_n, 2)) - v_0) / (2*acc_max);
		t_dcc = (v_0 - v_n) / acc_max + t_acc;
		t_const_v = 0;
		p_const_v = 0;
		v_peak = t_acc * acc_max;
		// if didn't manage to reach v_n
		if (t_dcc < 0.0) {
			t_acc = (sqrtf(2*acc_max*p + powf(v_0,2)) - v_0) / acc_max;
			t_dcc = 0;

			p_acc = (acc_max * powf(t_acc, 2) / 2) + (v_0 * t_acc);
			p_dcc = 0;

			// give corrected v_n value
			*v_n_ptr = acc_max * t_acc;
			success = false;
		}
	}

	path->t_phases[0] = t_acc;
	path->t_phases[1] = t_acc + t_const_v;
	path->t_phases[2] = t_acc + t_const_v + t_dcc;

	float p_phases[3];
	p_phases[0] = p_acc;
	p_phases[1] = p_acc + p_const_v;
	p_phases[2] = p_acc + p_const_v + p_dcc;

	quad_interp_init_acceleration(&(path->acc_interp), acc_max, 0, 0, v_0);
	lerp_init(&(path->const_v_interp), p_phases[0], p_phases[1], path->t_phases[0], path->t_phases[1]);
	quad_interp_init_acceleration(&(path->dcc_interp), -acc_max, path->t_phases[1], p_phases[1], v_peak);

	return success;
}

float path_calc_p(const Path *path, float t) {
    // acceleration
    if (t < path->t_phases[0]) {
        return quad_interp(&(path->acc_interp), t);
    }
    // constant speed
    else if (t < path->t_phases[1]) {
        return lerp(&(path->const_v_interp), t);
    }
    // decceleration
    else if (t <= path->t_phases[2]) {
        return quad_interp(&(path->dcc_interp), t);
    }
    // out of upper bound
    else {
        // return target p
        return quad_interp(&(path->dcc_interp), path->t_phases[2]);
    }
}
