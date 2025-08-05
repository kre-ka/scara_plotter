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

#endif /* INC_CURVE_H_ */