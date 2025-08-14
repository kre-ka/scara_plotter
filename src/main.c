#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "manipulator.h"
#include "curve.h"
#include "trajectory.h"


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

    // create bezier curve path
    float points[4][2] = {{100, 50}, {0, 0}, {200, 0}, {100, -50}};
    CubicCurve curve;
    cubic_curve_init_bezier(&curve, points);

    // create time-spatial trajectory from curve
    float (*xy_tab_traj)[2];
    int tab_traj_size;
    float v_n = 0.0;
    make_trajectory_single_curve(
        &xy_tab_traj, 
        &tab_traj_size, 
        &curve, 
        0.02, 
        0.0, 
        V_TARGET_DEFAULT, 
        &v_n, 
        ACC_MAX_DEFAULT);

    printf("\n");
    printf("trajectory points\nx\ty\n");
    for (int i=0; i <tab_traj_size; i++) {
        printf("%f\t%f\n", xy_tab_traj[i][0], xy_tab_traj[i][1]);
    }

    // check if all points are in manipulator range
    // TODO: maybe don't check all the points
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

    free(xy_tab_traj);
    
    return 0;
}