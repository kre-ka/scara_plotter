#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "curve.h"
#include "manipulator.h"
#include "trajectory.h"

#define RAD_TO_DEG 180 / M_PI
#define DEG_TO_RAD M_PI / 180

int main() {
  // initialize manipulator
  Manipulator manipulator;
  // right
  manipulator_init(&manipulator, 100.0, 100.0, -170.0 * DEG_TO_RAD, 0.0, 0.0,
                   150.0 * DEG_TO_RAD);
  // left
  // manipulator_init(&manipulator, 100.0, 100.0, 0.0, 170.0*DEG_TO_RAD,
  // -150.0*DEG_TO_RAD, 0.0);

  // // check work area
  // manipulator_print_work_area(&manipulator);

  // create bezier curve path
  float points[4][2] = {{100, 50}, {0, 0}, {200, 0}, {100, -50}};
  CubicCurve curve;
  cubic_curve_init_bezier(&curve, points);

  // create time-spatial trajectory from curve
  Trajectory trajectory;
  float v_n = 0.0;
  trajectory_init(&trajectory, curve, 1e-2, 0.0, V_TARGET_DEFAULT, &v_n,
                  ACC_MAX_DEFAULT);

  // execute trajectory one point at the time
  float t_res = 0.02;
  int i = 0;
  float t = i * t_res;
  float xy[2];
  float theta[2];
  printf("\n");
  while (t < trajectory.t_phases[2]) {
    trajectory_get_xy(&xy, &trajectory, t);
    printf("t: %f\tx: %f\ty: %f\t", t, xy[0], xy[1]);

    // check if point is in manipulator range
    // TODO: maybe don't check all the points
    if (!is_in_range_work_area(&manipulator, xy)) {
      printf("Point (%f, %f) out of manipulator range.\n", xy[0], xy[1]);
      return 1;
    }

    // calculate manipulator joint angles
    inverse_kinematics(theta, &manipulator, xy);
    printf("t0: %f\t t1: %f", theta[0], theta[1]);

    printf("\n");

    i++;
    t = i * t_res;
  }
  trajectory_free(&trajectory);

  return 0;
}