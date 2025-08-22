#include "manipulator.h"

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

bool manipulator_init(Manipulator *manipulator, float l_0, float l_1,
                      float theta_0_min, float theta_0_max, float theta_1_min,
                      float theta_1_max) {
  // validate input
  bool result = true;
  if (l_0 <= 0 || l_1 <= 0) {
    result = false;
    printf("l_0 and l_1 must be greater than 0.\n");
  }
  if (theta_0_max < theta_0_min || theta_1_max < theta_1_min) {
    result = false;
    printf("Maximal theta values must be greater than minial values.\n");
  }
  if (!result) return false;

  // set manipulator configuration based on theta_1 range
  // it will only work in one configuration (for simplicity)
  if (fabsf(theta_1_max) >= fabsf(theta_1_min))
    manipulator->configuration = RIGHT;
  else
    manipulator->configuration = LEFT;

  manipulator->l_0 = l_0;
  manipulator->l_1 = l_1;
  manipulator->theta_0_min = theta_0_min;
  manipulator->theta_0_max = theta_0_max;
  manipulator->theta_1_min = theta_1_min;
  manipulator->theta_1_max = theta_1_max;

  // work area data
  // these are helper variables to avoid duplicating code for both
  // configurations
  float theta_0_abs_min, theta_0_abs_max, theta_1_abs_min, theta_1_abs_max;
  if (manipulator->configuration == RIGHT) {
    theta_0_abs_min = theta_0_max;
    theta_0_abs_max = theta_0_min;
    theta_1_abs_min = theta_1_min;
    theta_1_abs_max = theta_1_max;
  } else {
    theta_0_abs_min = theta_0_min;
    theta_0_abs_max = theta_0_max;
    theta_1_abs_min = theta_1_max;
    theta_1_abs_max = theta_1_min;
  }
  // minimal x value (<=0)
  float x_min = l_0 * cosf(theta_0_abs_max) +
                l_1 * cosf(theta_0_abs_max + theta_1_abs_max);
  if (x_min < 0.0) x_min = 0.0;
  manipulator->work_area_data.x_min = x_min;
  // minimal range radius squared (center at (0,0))
  manipulator->work_area_data.r_min_sqr =
      powf(l_0, 2) + powf(l_1, 2) -
      2 * l_0 * l_1 * cosf(M_PI - theta_1_abs_max);
  // maximal range radius squared for theta_1=0 (manipulator fully straight;
  // center at (0,0))
  manipulator->work_area_data.r_max_straight_sqr =
      powf(l_0 + l_1 * cosf(theta_1_abs_min), 2);
  // maximal range radius squared for theta_0=max (moving theta_1 when theta_0
  // is at maximum position; center at (x_center_edge, y_center_edge))
  manipulator->work_area_data.r_max_edge_sqr = powf(l_1, 2);
  // center of maximal range circle for theta_0=max
  manipulator->work_area_data.x_center_edge = l_0 * cosf(theta_0_abs_min);
  manipulator->work_area_data.y_center_edge = l_0 * sinf(theta_0_abs_min);
  // border between r_max_straight and r_max_edge appliance area (based on last
  // point where manipulator can be fully straight)
  manipulator->work_area_data.y_border = (l_0 + l_1) * sinf(theta_0_abs_min);

  return true;
}

void manipulator_print_work_area(const Manipulator *manipulator) {
  const ManipulatorWorkAreaData *work_area = &(manipulator->work_area_data);
  printf("--- MANIPULATOR WORK AREA ---\n");
  printf("x >= %.2f\n", work_area->x_min);
  printf("x^2 + y^2 >= %.2f^2\n", sqrtf(work_area->r_min_sqr));
  if (manipulator->configuration == RIGHT) {
    printf("x^2 + y^2 <= %.2f^2 for y <= %.2f\n",
           sqrtf(work_area->r_max_straight_sqr), work_area->y_border);
    printf("(x - %.2f)^2 + (y - %.2f)^2 <= %.2f^2 for y > %.2f\n",
           work_area->x_center_edge, work_area->y_center_edge,
           sqrtf(work_area->r_max_edge_sqr), work_area->y_border);
  } else {
    printf("x^2 + y^2 <= %.2f^2 for y >= %.2f\n",
           sqrtf(work_area->r_max_straight_sqr), work_area->y_border);
    printf("(x - %.2f)^2 + (y - %.2f)^2 <= %.2f^2 for y < %.2f\n",
           work_area->x_center_edge, work_area->y_center_edge,
           sqrtf(work_area->r_max_edge_sqr), work_area->y_border);
  }
  printf("---\n");
}

bool is_in_range_angle(const Manipulator *manipulator, const float theta[2]) {
  return theta[0] >= manipulator->theta_0_min &&
         theta[0] <= manipulator->theta_0_max &&
         theta[1] >= manipulator->theta_1_min &&
         theta[1] <= manipulator->theta_1_max;
}

bool is_in_range_work_area(const Manipulator *manipulator,
                           const float point[2]) {
  // x >= x_min
  if (point[0] < manipulator->work_area_data.x_min) return false;
  float r_sqr = powf(point[0], 2) + powf(point[1], 2);
  // r_sqr >= r_min_sqr
  if (r_sqr < manipulator->work_area_data.r_min_sqr) return false;
  // (y <= y_border) XOR (config == RIGHT)
  // looks scary, but otherwise i would have to duplicate code
  if ((point[1] <= manipulator->work_area_data.y_border) !=
      (manipulator->configuration == RIGHT)) {
    // r_sqr <= r_max_straight_sqr
    if (r_sqr > manipulator->work_area_data.r_max_straight_sqr) return false;
  } else {
    // r_sqr <= r_max_edge_sqr
    if (powf(point[0] - manipulator->work_area_data.x_center_edge, 2) +
            powf(point[1] - manipulator->work_area_data.y_center_edge, 2) >
        manipulator->work_area_data.r_max_edge_sqr)
      return false;
  }
  return true;
}

void inverse_kinematics(float out[2], const Manipulator *manipulator,
                        const float in[2]) {
  float r = sqrtf(powf(in[0], 2) + powf(in[1], 2));
  out[0] = atan2f(in[1], in[0]) -
           manipulator->configuration *
               acosf((powf(manipulator->l_0, 2) - powf(manipulator->l_1, 2) +
                      powf(r, 2)) /
                     (2 * manipulator->l_0 * r));
  out[1] = manipulator->configuration *
           acosf((powf(r, 2) - powf(manipulator->l_0, 2) -
                  powf(manipulator->l_1, 2)) /
                 (2 * manipulator->l_0 * manipulator->l_1));
}
