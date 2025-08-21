#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include <stdbool.h>

#include "curve.h"
#include "interpolation.h"


typedef struct {
    CubicCurve path;
    float (*p_tau_map)[2];  // table for p(tau) mapping function: p (path length from the start) value, tau (path curve parameter) value
    int p_tau_map_size;
    float t_phases[3];  // movement phase change moments: stop accelerating, start deccelerating, end trajectory
    QuadInterp acc_interp;  // interpolator for acceleration phase
    Lerp const_v_interp;  // interpolator for constant speed phase
    QuadInterp dcc_interp;  // interpolator for decceleration phase
} Trajectory;

/*
Returns false and corrects value under v_f_ptr if didn't manage to reach desired v_f (probably will never happen though).
Follow with call to trajectory_free when you're done using it.

Params:
- trajectory
- path - path to follow
- abs_err_max - absolute path following accuracy [mm]
- rel_error_max - relative path following accuracy
- v_0 - intial speed [mm/s]
- v_target - target speed [mm/s]
- v_f_ptr - final speed [mm/s] (can be corrected inside function, hence pointer)
- acc - acceleration and decceleration [mm/s^2]
*/
bool trajectory_init(Trajectory *trajectory, CubicCurve path, float abs_err_max, float rel_error_max, float v_0, float v_target, float *v_f_ptr, float acc);

/*
Frees dynamic memory allocated by trajectory
*/
void trajectory_free(Trajectory *trajectory);

void trajectory_get_xy(float (*out_xy)[2], const Trajectory *trajectory, float t);

float _trajectory_calc_p(const Trajectory *trajectory, float t);

/*
High level function to make time-spatial ((x, y)(t)) trajectory from spatial path defined by single curve.

/*
Caluculates Trajectory t_phases and interpolators. 
Returns false and corrects value under v_f_ptr if didn't manage to reach desired v_f (probably will never happen though).

Params:
- trajectory
- v_0 - initial speed [mm/s]
- v_target - target speed [mm/s]
- v_f_ptr - final speed [mm/s] (can be corrected inside function, hence pointer)
- acc - acceleration (and decceleration) [mm/s^2]
*/
bool _calc_t_phases_interpolators(Trajectory *trajectory, float v_0, float v_target, float *v_f_ptr, float acc);

#endif /* INC_TRAJECTORY_H_ */