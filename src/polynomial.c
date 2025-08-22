#include "polynomial.h"

#include <math.h>

float poly_eval_f(float t, int degree, const float coef[degree + 1]) {
  float out = 0.0;
  for (int i = 0; i <= degree; i++) {
    out += powf(t, degree - i) * coef[i];
  }
  return out;
}