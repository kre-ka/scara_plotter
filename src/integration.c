#include "integration.h"

float integrate_step_trapezoid(const float x[2], const float t[2]){
    float out = (x[1] + x[0]) * (t[1] - t[0]) / 2;

    return out;
}