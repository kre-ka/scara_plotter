#include "integration.h"

void integrate_trapezoid(float *out, int num_points, float *t, float *x){
    out[0] = 0.0;
    for (int i=1; i < num_points; i++){
        out[i] = out[i-1] + (t[i] - t[i-1]) * (x[i] + x[i-1]) / 2;
    }
}