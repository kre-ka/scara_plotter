#ifndef INC_CURVE_H_
#define INC_CURVE_H_

typedef struct {
	float coef[2][4];
	float t_span[2];
	float deg;
} CubicCurve;

typedef struct {
	float coef[2][3];
	float t_span[2];
	float deg;
} QuadraticCurve;


void cubic_curve_init_bezier(CubicCurve *curve, const float points[4][2]);

void cubic_curve_diff(QuadraticCurve *out, const CubicCurve *in);

float p_integrand_fun(float t, int poly_deg, const float curve_diff_coef[2][poly_deg+1]);

/*
Populates tables for p(t) mapping function - curve length (p) for given curve parameter (t).
The function is expected to be a linear interpolation between table points.

Parameters:
- out_p - curve length (distance from the start) table
- out_t - curve parameter
- size_ptr - number of table elements
- curve - curve to follow
- abs_err_max - maximum allowed absolute error [mm]
- rel_error_max - maximum allowed relative error
*/
void make_p_t_map_tables(float **out_p, float **out_t, int *size_ptr, const CubicCurve *curve, float abs_err_max, float rel_error_max);

#endif /* INC_CURVE_H_ */