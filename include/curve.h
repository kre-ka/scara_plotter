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

/*
Returns rate of curve length change (dp/dt) for given curve parameter.

Parameters:
- t - curve parameter value
- poly_deg - curve polynomial degree
- curve_diff_coef - curve differential coefficients
*/
float dp_dt_fun(float t, int poly_deg,
                const float curve_diff_coef[2][poly_deg + 1]);

/*
Populates table for t(p) mapping function - curve parameter (t) for given curve
length (p). The function is expected to be a linear interpolation between table
points.

Parameters:
- out_p_t_dyn - curve length (distance from the start) and corresponding curve
parameter table; it is pointing to dynamically allocated memory
- size_ptr - number of table rows
- curve - curve to follow
- abs_err_max - maximum allowed absolute error [mm]
*/
void make_p_t_map_table(float (**out_p_t_dyn)[2], int *out_size_ptr,
                        const CubicCurve *curve, float abs_err_max);

#endif /* INC_CURVE_H_ */