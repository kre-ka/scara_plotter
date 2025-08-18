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

float dp_dt_fun(float t, int poly_deg, const float curve_diff_coef[2][poly_deg+1]){
    float curve_diff_eval[2];
    curve_diff_eval[0] = poly_eval_f(t, curve_diff_coef[0], poly_deg);
    curve_diff_eval[1] = poly_eval_f(t, curve_diff_coef[1], poly_deg);

    return sqrtf(powf(curve_diff_eval[0], 2) + powf(curve_diff_eval[1], 2));
}

void make_p_t_map_table(float (**out_p_t)[2], int *size_ptr, const CubicCurve *curve, float abs_err_max, float rel_error_max){
    // curve differential is needed for path length computations
    QuadraticCurve curve_diff;
    cubic_curve_diff(&curve_diff, curve);

    // find `t` values between which `p` changes (approximately) linearly
    float *t_tab;
    int tab_size;
    find_interpolation_points_linear(&t_tab, &tab_size, curve_diff.deg+1, dp_dt_fun, curve_diff.coef, curve_diff.t_span, abs_err_max, rel_error_max);
    
    // // calculate `p` values for given `t`s by integrating rates of `p` change (dp/dt)
    float *p_tab;
    p_tab = malloc(tab_size * sizeof(float));
    // set initial `p` value (note that the loop starts on 1)
    p_tab[0] = 0.0;
    // initialize dp_dt data
    // only 2 samples are needed at the time, no need to store all of them
    float dp_dt[2];
    dp_dt[1] = dp_dt_fun(t_tab[0], curve_diff.deg, curve_diff.coef);
    for (int i=1; i < tab_size; i++){
        // move integrand data one sample forward
        dp_dt[0] = dp_dt[1];
        dp_dt[1] = dp_dt_fun(t_tab[i], curve_diff.deg, curve_diff.coef);

        p_tab[i] = p_tab[i-1] + integrate_step_trapezoid(dp_dt, &t_tab[i-1]);
    }

    printf("path length: %f\n", p_tab[tab_size-1]);

    *out_p_t = malloc(tab_size * 2 * sizeof(float));
    for (int i=0; i < tab_size; i++) {
        (*out_p_t)[i][0] = p_tab[i];
        (*out_p_t)[i][1] = t_tab[i];
    }
    *size_ptr = tab_size;

    free(t_tab);
    free(p_tab);
}