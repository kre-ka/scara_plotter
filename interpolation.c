#include "interpolation.h"

void lerp_init(Lerp *lerp, float x_0, float x_n, int t_n){
    lerp->x_0 = x_0;
    lerp->step = (float) (x_n - x_0) / t_n;
    lerp->t_n = t_n;
}

float lerp(Lerp *lerp, int t){
    return lerp->x_0 + t * lerp->step;
}