#ifndef INC_INTERPOLATION_H_
#define INC_INTERPOLATION_H_

#include "data_structs.h"

typedef struct {
	float x_0;
	float t_0;
	float step;
} Lerp;

void lerp_init(Lerp *lerp, float x_0, float x_n, float t_0, float t_n);
float lerp(const Lerp *lerp, float t);

typedef struct {
    float a, b, c;
} QuadInterp;

void quad_interp_init(QuadInterp *interp, const float p_0[2], const float p_1[2], const float p_2[2]);
void quad_interp_init_with_coef(QuadInterp *interp, float a, float b, float c);
void quad_interp_init_acceleration(QuadInterp *interp, float acc, float t_0, float x_0, float v_0);
float quad_interp(const QuadInterp *interp, float t);

struct _find_interpolation_points_extra_params{
    float abs_err_max;
    float rel_error_max;
    float test_points[3];
};

void find_interpolation_points_linear(float **t_out_dyn, int *t_tab_size, int f_param_num, const float (*f)(float, int, const float [2][f_param_num]), const float f_params[2][f_param_num], const float t_span[2], float abs_err_max, float rel_error_max);

void _find_interpolation_points_linear(int f_param_num, const float (*f)(float, int, const float [2][f_param_num]), const float f_params[2][f_param_num], const struct _find_interpolation_points_extra_params *extra_params, const float t_tab[2], const float f_tab[2], TreeNode *t_out_root);

/*
Returns value mapped from t using linear interpolation between t_x_map lookup table points.

Parameters:
- t - input value
- t_x_map - lookup table of corresponding t (input) and x (output) values
- map_size - number of t_x_map rows (t_x pairs)
*/
float lerp_map(float t, const float (**t_x_map)[2], int map_size);

/*
Returns value mapped from t using linear interpolation between t_x_map lookup table points.
Accelerates consecutive lerps when t values are ascending. Can only be used with ascending t. 

Parameters:
- t - input value
- lerp_s - Lerp structure - to be preserved between calls
- idx_map_ptr - current lookup table index - to be preserved between calls
- t_x_map - lookup table of corresponding t (input) and x (output) values
- map_size - number of t_x_map rows (t_x pairs)
*/
float lerp_map_ascending_optimized(float t, Lerp *lerp_s, int *idx_map_ptr, const float (**t_x_map)[2], int map_size);

#endif /* INC_INTERPOLATION_H_ */