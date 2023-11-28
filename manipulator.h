#ifndef INC_MANIPULATOR_H_
#define INC_MANIPULATOR_H_

#include <stdbool.h>

// movement speed target [mm/s]
#define V_TARGET 100.0
// maximum acceleration (and decceleration) [mm/s^2]
#define ACC_MAX 1000.0

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

/*
Calculates movement phase change moments for a single curve, that is when to:
- stop accelerating
- start deccelerating
- end movement

Returns false and corrects v_n value if didn't manage to reach v_n (probably will never happen though).

params:
- t_phases_out - movement phase change moments output in seconds [s]
- p_n - path lenght in millimeters [mm]
- v_0 - initial speed [mm/s]
- v_n - final speed [mm/s]
- v_n_corrected - pointer to correct v_n if necessary
*/
bool calc_movement_time(float t_phases_out[3], float p, float v_0, float v_n, float *v_n_corrected);

/*
Calculates p(t) (position in time) trajectory in curve space based on initial speed and movement phase changes form calc_movement_time

params:
- p_tab_out - pointer to array of positions along curve in subsequent time moments in millimeters [mm]
- t_phases - movement phase change moments from calc_movement_time in seconds [s]
- t_res - time resolution [s]
- p - final position (to avoid going too far) [mm]
- v_0 - initial speed [mm/s]
*/
void calc_p_trajectory(float *p_tab_out, float t_phases[3], float t_res, float p, float v_0);

#endif /* INC_MANIPULATOR_H_ */
