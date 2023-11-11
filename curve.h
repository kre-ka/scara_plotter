#ifndef INC_CURVE_H_
#define INC_CURVE_H_

struct cubic_curve {
	float coef[2][4];
	float t_span[2];
	float deg;
};
typedef struct cubic_curve CubicCurve;

struct quadratic_curve {
	float coef[2][3];
	float t_span[2];
	float deg;
};
typedef struct quadratic_curve QuadraticCurve;


void cubic_curve_init_bezier(CubicCurve *curve, float points[4][2]);

void cubic_curve_diff(QuadraticCurve *out, CubicCurve *in);

#endif /* INC_CURVE_H_ */