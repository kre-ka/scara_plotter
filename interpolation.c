#include "interpolation.h"
#include <math.h>
#include <stdlib.h>


void lerp_init(Lerp *lerp, float x_0, float x_n, int t_n){
    lerp->x_0 = x_0;
    lerp->step = (float) (x_n - x_0) / t_n;
    lerp->t_n = t_n;
}

float lerp(Lerp *lerp, int t){
    return lerp->x_0 + t * lerp->step;
}

void find_interpolation_points_linear(float **t_out_dyn, int *t_tab_size, int f_param_num, float (*f)(float, int, float [2][f_param_num]), float f_params[2][f_param_num], float t_span[2], float abs_err_max, float rel_error_max){
    struct _find_interpolation_points_extra_params params;
    params.abs_err_max = abs_err_max;
    params.rel_error_max = rel_error_max;
    
    params.test_points[0] = 0.25;
    params.test_points[1] = 0.5;
    params.test_points[2] = 0.75;

    TreeNode *t_tab_root = tree_create_node(0);

    tree_add_node_left(t_tab_root, t_span[0]);
    tree_add_node_right(t_tab_root, t_span[1]);

    float t_tab_init[] = {t_span[0], (t_span[0]+t_span[1])/2, t_span[1]};
    float f_tab_init[3];
    for (int i=0; i < 3; i++){
        f_tab_init[i] = (*f)(t_tab_init[i], f_param_num-1, f_params);
    }

    _find_interpolation_points_linear(f_param_num, f, f_params, &params, t_tab_init, f_tab_init, t_tab_root->left);

    int size = 0;
    tree_leaves_to_array_get_size(t_tab_root, &size);
    *t_tab_size = size;
    *t_out_dyn = malloc(size * sizeof(float));
    if (!*t_out_dyn) exit(EXIT_FAILURE);
    int idx = 0;
    tree_leaves_to_array(t_tab_root, *t_out_dyn, &idx);
    tree_free(t_tab_root);
}

void _find_interpolation_points_linear(int f_param_num, float (*f)(float, int, float [2][f_param_num]), float f_params[2][f_param_num], struct _find_interpolation_points_extra_params *extra_params, float t_tab[2], float f_tab[2], TreeNode *t_out_root){
    float t_test[3];
    float f_test_true[3];
    float f_test_interpolated[3];

    t_test[0] = t_tab[0] + (t_tab[2]-t_tab[0]) * extra_params->test_points[0];
    t_test[1] = t_tab[1];
    t_test[2] = t_tab[0] + (t_tab[2]-t_tab[0]) * extra_params->test_points[2];

    f_test_true[0] = (*(f))(t_test[0], f_param_num-1, f_params);
    f_test_true[1] = f_tab[1];
    f_test_true[2] = (*(f))(t_test[2], f_param_num-1, f_params);
    
    for (int i=0; i < 3; i++){
        f_test_interpolated[i] = f_tab[0] + (f_tab[2]-f_tab[0]) * extra_params->test_points[i];
    }

    for (int i=0; i < 3; i++){
        float abs_error = fabs(f_test_interpolated[i] - f_test_true[i]);
        float rel_error = abs_error / f_test_true[i];
        if (abs_error > extra_params->abs_err_max || rel_error > extra_params->rel_error_max){
            float t_tab_new[] = {t_tab[0], t_test[0], t_test[1]};
            float f_tab_new[] = {f_tab[0], f_test_true[0], f_test_true[1]};
            tree_add_node_left(t_out_root, t_tab_new[0]);
            _find_interpolation_points_linear(f_param_num, f, f_params, extra_params, t_tab_new, f_tab_new, t_out_root->left);
            
            t_tab_new[0] = t_test[1];
            t_tab_new[1] = t_test[2];
            t_tab_new[2] = t_tab[2];
            f_tab_new[0] = f_test_true[1];
            f_tab_new[1] = f_test_true[2];
            f_tab_new[2] = f_tab[2];
            tree_add_node_right(t_out_root, t_tab_new[0]);
            _find_interpolation_points_linear(f_param_num, f, f_params, extra_params, t_tab_new, f_tab_new, t_out_root->right);
            break;
        }
    }
}