#include "curve.h"
#include "polynomial.h"
#include "interpolation.h"
#include "integration.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

void cubic_curve_init_bezier(CubicCurve *curve, const float points[4][2]){
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

void cubic_curve_diff(QuadraticCurve *out, const CubicCurve *in){
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

float p_integrand_fun(float t, int poly_deg, const float curve_diff_coef[2][poly_deg+1]){
    float curve_diff_eval[2];
    curve_diff_eval[0] = poly_eval_f(t, curve_diff_coef[0], poly_deg);
    curve_diff_eval[1] = poly_eval_f(t, curve_diff_coef[1], poly_deg);

    return sqrtf(powf(curve_diff_eval[0], 2) + powf(curve_diff_eval[1], 2));
}

void make_p_t_map_tables(float **out_p, float **out_t, int *size_ptr, const CubicCurve *curve, float abs_err_max, float rel_error_max){
    // curve differential is needed for path length computations
    QuadraticCurve curve_diff;
    cubic_curve_diff(&curve_diff, curve);

    // find `t` values between which `p` changes (approximately) linearly
    float *t_tab;
    int tab_size;
    find_interpolation_points_linear(&t_tab, &tab_size, curve_diff.deg+1, p_integrand_fun, curve_diff.coef, curve_diff.t_span, abs_err_max, rel_error_max);
    
    // calculate `p` values for given `t`s
    // first, calculate `p` integrands - rates of `p` change (dp/dt)
    // TODO: remove this table - get p_integrand step by step while calculating p
    float p_integrand_tab[tab_size];
    for (int i=0; i < tab_size; i++){
        p_integrand_tab[i] = p_integrand_fun(t_tab[i], curve_diff.deg, curve_diff.coef);
    }
    // then integrate them into `p` values
    float *p_tab;
    p_tab = malloc(tab_size * sizeof(float));
    integrate_trapezoid(p_tab, tab_size, t_tab, p_integrand_tab);

    *out_p = p_tab;
    *out_t = t_tab;
    *size_ptr = tab_size;
    
    printf("path length: %f\n", p_tab[tab_size-1]);
}