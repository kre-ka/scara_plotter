#ifndef INC_PATH_H_
#define INC_PATH_H_

#include <stdbool.h>
#include "interpolation.h"

typedef struct {
    QuadInterp acc_interp;
    Lerp const_v_interp;
    QuadInterp dcc_interp;
    float t_phases[3];
} Path;

/*
Params:
- path - pointer to Path struct
- p - path lenght in millimeters [mm]
- v_0 - initial speed [mm/s]
- v_n_ptr - pointer to final speed value [mm/s] (can be corrected inside function, hence pointer)
- v_target - target speed [mm/s]
- acc_max - maximum acceleration (and decceleration) [mm/s^2]

Returns false and corrects value under v_n_ptr if didn't manage to reach v_n (probably will never happen though).
*/
bool path_init(Path *path, float p, float v_0, float *v_n_ptr, float v_target, float acc_max);

float path_calc_p(const Path *path, float t);

#endif /* INC_PATH_H_ */