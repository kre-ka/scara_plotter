#ifndef INC_MANIPULATOR_H_
#define INC_MANIPULATOR_H_

#include <stdbool.h>
#include "curve.h"
#include "interpolation.h"

// movement speed target [mm/s]
#define V_TARGET_DEFAULT 100.0
// maximum acceleration (and decceleration) [mm/s^2]
#define ACC_MAX_DEFAULT 1000.0

typedef struct {
	float x_min, r_min_sqr, r_max_straight_sqr, r_max_edge_sqr, y_border, x_center_edge, y_center_edge;
} ManipulatorWorkAreaData;

typedef enum {
	LEFT = -1,
	RIGHT = 1	
} ManipulatorConfig;

typedef struct {
	float l_0;
	float l_1;
	float theta_0_min;
	float theta_0_max;
	float theta_1_min;
	float theta_1_max;
	ManipulatorConfig configuration;
	ManipulatorWorkAreaData work_area_data;
} Manipulator;

bool manipulator_init(Manipulator *manipulator, float l_0, float l_1, float theta_0_min, float theta_0_max, float theta_1_min, float theta_1_max);

void manipulator_print_work_area(Manipulator *manipulator);

bool is_in_range_angle(Manipulator *manipulator, float theta[2]);

bool is_in_range_work_area(Manipulator *manipulator, float point[2]);

void inverse_kinematics(Manipulator *manipulator , float in[2], float out[2]);

#endif /* INC_MANIPULATOR_H_ */
