#include "polynomial.h"
#include <math.h>

float poly_eval_f(float t, float *coef, int degree){
    float out = 0.0;
    for(int i=0; i <= degree; i++){
        out += powf(t, degree-i) * coef[i];
    }
    return out;
}