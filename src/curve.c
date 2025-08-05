#include "curve.h"
#include "polynomial.h"
#include <math.h>

void cubic_curve_init_bezier(CubicCurve *curve, float points[4][2]){
    for(int i=0; i < 2; i++){
        curve->coef[i][0] = -points[0][i] + 3*points[1][i] - 3*points[2][i] + points[3][i];
        curve->coef[i][1] = 3*points[0][i] - 6*points[1][i] + 3*points[2][i];
        curve->coef[i][2] = -3*points[0][i] + 3*points[1][i];
        curve->coef[i][3] = points[0][i];
    }
    curve->deg = 3;
    curve->t_span[0] = 0;
    curve->t_span[1] = 1;
}

void cubic_curve_diff(QuadraticCurve *out, CubicCurve *in){
    for(int i=0; i < 2; i++){
        out->coef[i][0] = 3*in->coef[i][0];
        out->coef[i][1] = 2*in->coef[i][1];
        out->coef[i][2] = in->coef[i][2];
    }
    for(int i=0; i <= 1; i++){
        out->t_span[i] = in->t_span[i];
    }
    out->deg = 2;
}

float p_integrand_fun(float t, int poly_deg, float curve_diff_coef[2][poly_deg+1]){
    float curve_diff_eval[2];
    curve_diff_eval[0] = poly_eval_f(t, curve_diff_coef[0], poly_deg);
    curve_diff_eval[1] = poly_eval_f(t, curve_diff_coef[1], poly_deg);

    return sqrtf(powf(curve_diff_eval[0], 2) + powf(curve_diff_eval[1], 2));
}