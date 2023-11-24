#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include "manipulator.h"
#include "curve.h"
#include "polynomial.h"
#include "interpolation.h"
#include "integration.h"


#define RAD_TO_DEG 180/M_PI
#define DEG_TO_RAD M_PI/180

int main(){
    // initialize manipulator
    Manipulator manipulator;
    // right
    manipulator_init(&manipulator, 100.0, 100.0, -170.0*DEG_TO_RAD, 0.0, 0.0, 150.0*DEG_TO_RAD);
    // left
    // manipulator_init(&manipulator, 100.0, 100.0, 0.0, 170.0*DEG_TO_RAD, -150.0*DEG_TO_RAD, 0.0);

    // // check work area
    // manipulator_print_work_area(&manipulator);

    // create a bezier curve trajectory
    CubicCurve curve;
    float points[4][2] = {{100, 50}, {0, 0}, {200, 0}, {100, -50}};
    cubic_curve_init_bezier(&curve, points);
    // curve differential is needed for computations
    QuadraticCurve curve_diff;
    cubic_curve_diff(&curve_diff, &curve);

    // find tau_tab_map - points needed to linearly approximate p_integrand(tau) function, where tau is curve parameter and p_integrand is rate of curve length change
    // this will be used to calculate curve length (p) and tau(p) function
    float *tau_tab_map_dyn;
    int tab_map_size;
    find_interpolation_points_linear(&tau_tab_map_dyn, &tab_map_size, curve_diff.deg+1, p_integrand_fun, curve_diff.coef, curve_diff.t_span, 1e-2, 1e-2);
    
    // this is just to move t_tab to stack, as I would probably forget to free it later
    float tau_tab_map[tab_map_size];
    for (int i=0; i < tab_map_size; i++){
        tau_tab_map[i] = tau_tab_map_dyn[i];
    }
    free(tau_tab_map_dyn);

    // calculate p_tab_map - curve length (p) for tau given in tau_tab_map
    float p_integrand_tab[tab_map_size];
    float p_tab_map[tab_map_size];
    for (int i=0; i < tab_map_size; i++){
        p_integrand_tab[i] = p_integrand_fun(tau_tab_map[i], curve_diff.deg, curve_diff.coef);
    }
    integrate_trapezoid(p_tab_map, tab_map_size, tau_tab_map, p_integrand_tab);
    printf("path length: %f\n", p_tab_map[tab_map_size-1]);

    // find t_phases - time moments to stop acceleration, to start decceleration and end movement to traverse curve, 
    // given initial and final speed given here, and acceleration and target speed defined in manipulator.h
    float v_0 = 0;
    float v_n = 0;
    float t_phases[3];
    // it's possible to not reach final speed when curve is short and acceleration is low
    // in this case final speed is corrected and should be considered for next curve
    if (!calc_movement_time(t_phases, p_tab_map[tab_map_size-1], v_0, v_n, &v_n)){
        printf("End speed cannot be reached.\n");
    }
    printf("movement phases moments: %f, %f, %f\n\n", t_phases[0], t_phases[1], t_phases[2]);

    // calculate p_tab_traj - distance trajectory for equally spaced time samples, defined by t_res - time resolution in seconds
    float t_res = 0.02;
    int tab_traj_size = (int) ceilf(t_phases[2] / t_res);
    float p_tab_traj[tab_traj_size];
    calc_p_trajectory(p_tab_traj, t_phases, t_res, p_tab_map[tab_map_size-1], v_0);

    // map p_tab_traj into tau_tab_traj - tau values for given time samples, linearly interpolated between map points calculated earlier
    float tau_tab_traj[tab_traj_size];
    lerp_map(tau_tab_traj, p_tab_traj, tab_traj_size, p_tab_map, tau_tab_map, tab_map_size);

    // evaluate (x, y) coordinates for tau_tab_traj
    float xy_tab_traj[tab_traj_size][2];
    for (int i=0; i < tab_traj_size; i++) {
        xy_tab_traj[i][0] = poly_eval_f(tau_tab_traj[i], curve.coef[0], curve.deg);
        xy_tab_traj[i][1] = poly_eval_f(tau_tab_traj[i], curve.coef[1], curve.deg);
    }

    // check if all points are in manipulator range
    for (int i=0; i < tab_traj_size; i++) {
        if (!is_in_range_work_area(&manipulator, xy_tab_traj[i])) {
            printf("Point (%f, %f) out of manipulator range.\n", xy_tab_traj[i][0], xy_tab_traj[i][1]);
            return 1;
        }
    }
    
    // calculate inverse kinematics (manipulator joint angles) for trajectory points
    float theta_tab_traj[tab_traj_size][2];
    for (int i=0; i < tab_traj_size; i++) {
        inverse_kinematics(&manipulator, xy_tab_traj[i], theta_tab_traj[i]);
    }
    
    return 0;
}