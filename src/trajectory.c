#include "trajectory.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "curve.h"
#include "debug.h"
#include "integration.h"
#include "interpolation.h"
#include "polynomial.h"

bool trajectory_init(Trajectory* trajectory, CubicCurve path,
                     float err_max_speed, float v_0, float v_target,
                     float* v_f_ptr, float acc) {
  trajectory->path = path;

  make_p_t_map_table(&(trajectory->p_tau_map), &(trajectory->p_tau_map_size),
                     &(trajectory->path), err_max_speed);

  // it's possible to not reach final speed when curve is short and
  // acceleration is low in this case final speed is corrected and should be
  // considered for next curve
  bool success = true;
  if (!_calc_t_phases_interpolators(trajectory, v_0, v_target, v_f_ptr, acc)) {
    success = false;
    printf("End speed cannot be reached.\n");
  }
  debug_print("movement phases moments: %f, %f, %f\n", trajectory->t_phases[0],
              trajectory->t_phases[1], trajectory->t_phases[2]);

  return success;
}

void trajectory_free(Trajectory* trajectory) { free(trajectory->p_tau_map); }

void trajectory_get_xy(float (*out_xy)[2], const Trajectory* trajectory,
                       float t) {
  // get path length p for time t
  float p = _trajectory_calc_p(trajectory, t);

  // get curve parameter tau for p
  Lerp lerp;
  int idx_map = 0;
  // can't give this guy const p_tau_map
  float tau = lerp_map_ascending_optimized(
      p, &(trajectory->p_tau_map), trajectory->p_tau_map_size, &lerp, &idx_map);

  // get x, y coordinates for tau
  (*out_xy)[0] =
      poly_eval_f(tau, trajectory->path.deg, trajectory->path.coef[0]);
  (*out_xy)[1] =
      poly_eval_f(tau, trajectory->path.deg, trajectory->path.coef[1]);
}

float _trajectory_calc_p(const Trajectory* trajectory, float t) {
  // acceleration
  if (t < trajectory->t_phases[0]) {
    return quad_interp(&(trajectory->acc_interp), t);
  }
  // constant speed
  else if (t < trajectory->t_phases[1]) {
    return lerp(&(trajectory->const_v_interp), t);
  }
  // decceleration
  else if (t <= trajectory->t_phases[2]) {
    return quad_interp(&(trajectory->dcc_interp), t);
  }
  // out of upper bound
  else {
    // return target p
    return quad_interp(&(trajectory->dcc_interp), trajectory->t_phases[2]);
  }
}

bool _calc_t_phases_interpolators(Trajectory* trajectory, float v_0,
                                  float v_target, float* v_f_ptr, float acc) {
  // full path length
  float p = trajectory->p_tau_map[trajectory->p_tau_map_size - 1][0];

  float v_f = *v_f_ptr;
  float t_acc, t_const_v,
      t_dcc;  // acceleration time, constant speed time, decceleration time
  float p_acc, p_const_v, p_dcc;  // similar, but for path length

  t_acc = (v_target - v_0) / acc;
  t_dcc = (v_target - v_f) / acc;

  p_acc = (acc * powf(t_acc, 2) / 2) + (v_0 * t_acc);
  p_dcc = (v_target * t_dcc) - (acc * powf(t_dcc, 2) / 2);
  p_const_v = p - p_acc - p_dcc;

  t_const_v = p_const_v / v_target;

  bool success = true;
  // if didn't manage to reach target speed
  if (p_const_v < 0.0) {
    t_acc = (sqrtf(4 * acc * p + 3 * powf(v_0, 2) - 4 * v_0 * v_f +
                   2 * powf(v_f, 2)) -
             v_0) /
            (2 * acc);
    t_dcc = (v_0 - v_f) / acc + t_acc;
    t_const_v = 0;
    p_const_v = 0;
    v_target = t_acc * acc;

    // if didn't manage to reach v_f
    if (t_dcc < 0.0) {
      t_acc = (sqrtf(2 * acc * p + powf(v_0, 2)) - v_0) / acc;
      t_dcc = 0;

      p_acc = (acc * powf(t_acc, 2) / 2) + (v_0 * t_acc);
      p_dcc = 0;

      v_f = acc * t_acc;

      // set corrected v_f value
      *v_f_ptr = v_f;
      success = false;
    }
  }

  trajectory->t_phases[0] = t_acc;
  trajectory->t_phases[1] = t_acc + t_const_v;
  trajectory->t_phases[2] = t_acc + t_const_v + t_dcc;

  float p_phases[3];
  p_phases[0] = p_acc;
  p_phases[1] = p_acc + p_const_v;
  p_phases[2] = p_acc + p_const_v + p_dcc;

  quad_interp_init_acceleration(&(trajectory->acc_interp), acc, 0, 0, v_0);
  lerp_init(&(trajectory->const_v_interp), p_phases[0], p_phases[1],
            trajectory->t_phases[0], trajectory->t_phases[1]);
  quad_interp_init_acceleration(&(trajectory->dcc_interp), -acc,
                                trajectory->t_phases[1], p_phases[1], v_target);

  return success;
}
