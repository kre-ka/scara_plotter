#ifndef INC_INTEGRATION_H_
#define INC_INTEGRATION_H_

/*
Returns integration step of function `x` over variable `t` using trapezoid rule.

Parameters:
- x - initial and final funciton values
- t - initial and final function variable values
*/
float integrate_step_trapezoid(const float x[2], const float t[2]);

#endif /* INC_INTEGRATION_H_ */