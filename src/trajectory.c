#include "trajectory.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "curve.h"
#include "polynomial.h"
#include "interpolation.h"
#include "integration.h"
#include "path.h"


void make_trajectory_single_curve(float (**out)[2], int *out_size_ptr, const CubicCurve *curve, float t_res, float v_0, float v_target, float *v_n_ptr, float acc_max){
    float *p_tab_map;
    float *tau_tab_map;
    int tab_map_size;
    make_p_t_map_tables(&p_tab_map, &tau_tab_map, &tab_map_size, curve, 1e-2, 1e-2);

    // find t_phases - time moments to change movement phase - stop acceleration, start decceleration and end movement, 
    // given initial and final speed, target speed and acceleration
    // TODO: evaluate trajectory points one at a time - to avoid creating large arrays of size unknown at compile time
    
    // make Path struct, from which p(t) trajectory will be constructed
    Path path;
    // it's possible to not reach final speed when curve is short and acceleration is low
    // in this case final speed is corrected and should be considered for next curve
    if (!path_init(&path, p_tab_map[tab_map_size-1], v_0, v_n_ptr, v_target, acc_max)){
        printf("End speed cannot be reached.\n");
    }
    printf("movement phases moments: %f, %f, %f\n\n", path.t_phases[0], path.t_phases[1], path.t_phases[2]);
    
    // calculate p_tab_traj - p trajectory for equally spaced time samples, defined by t_res - time resolution in seconds
    int tab_traj_size = (int) ceilf(path.t_phases[2] / t_res);
    float p_tab_traj[tab_traj_size];
    for (int i=0; i < tab_traj_size; i++) {
        p_tab_traj[i] = path_calc_p(&path, t_res*i);
    }

    // print p(t) trajectory
    for (int i=0; i < tab_traj_size; i++) {
        printf("%f\t%f\n", t_res*i, p_tab_traj[i]);
    }

    // map p_tab_traj into tau_tab_traj - tau values for given time samples, linearly interpolated between map points calculated earlier
    float tau_tab_traj[tab_traj_size];
    lerp_map(tau_tab_traj, p_tab_traj, tab_traj_size, p_tab_map, tau_tab_map, tab_map_size);

    free(p_tab_map);
    free(tau_tab_map);

    // evaluate (x, y) coordinates for tau_tab_traj
    float xy_tab_traj[tab_traj_size][2];
    for (int i=0; i < tab_traj_size; i++) {
        xy_tab_traj[i][0] = poly_eval_f(tau_tab_traj[i], curve->coef[0], curve->deg);
        xy_tab_traj[i][1] = poly_eval_f(tau_tab_traj[i], curve->coef[1], curve->deg);
    }

    *out = malloc(sizeof(xy_tab_traj));
    memcpy(*out, xy_tab_traj, sizeof(xy_tab_traj));
    *out_size_ptr = tab_traj_size;
}