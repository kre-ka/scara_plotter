#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include "curve.h"

/*
High level function to make time-spatial ((x, y)(t)) trajectory from spatial path defined by single curve.

Params:
- out - resulting trajectory - array of consecutive {x, y} points [mm]
- out_size - number of points in resulting trajectory
- curve - path to follow
- t_res - time resolution [s]
- v_0 - initial speed [mm/s]
- v_target - target speed [mm/s]
- v_n_ptr - pointer to desired final speed value [mm/s] (can be corrected inside function, hence pointer)
- acc_max - maximum acceleration (and decceleration) [mm/s^2]
*/
void make_trajectory_single_curve(float (**out)[2], int *out_size_ptr, const CubicCurve *curve, float t_res, float v_0, float v_target, float *v_n_ptr, float acc_max);

#endif /* INC_TRAJECTORY_H_ */