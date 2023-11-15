#ifndef INC_INTERPOLATION_H_
#define INC_INTERPOLATION_H_

#include "data_structs.h"

typedef struct {
	float x_0;
	float t_0;
	float step;
} Lerp;

void lerp_init(Lerp *lerp, float x_0, float x_n, float t_0, float t_n);
float lerp(Lerp *lerp, float t);


struct _find_interpolation_points_extra_params{
    float abs_err_max;
    float rel_error_max;
    float test_points[3];
};

void find_interpolation_points_linear(float **t_out_dyn, int *t_tab_size, int f_param_num, float (*f)(float, int, float [2][f_param_num]), float f_params[2][f_param_num], float t_span[2], float abs_err_max, float rel_error_max);

void _find_interpolation_points_linear(int f_param_num, float (*f)(float, int, float [2][f_param_num]), float f_params[2][f_param_num], struct _find_interpolation_points_extra_params *extra_params, float t_tab[2], float f_tab[2], TreeNode *t_out_root);

#endif /* INC_INTERPOLATION_H_ */