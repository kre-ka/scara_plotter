#include "fixed_point.h"

#include <math.h>

float fixed_to_float(int x, int scale) { return (float)x / (1 << scale); }

int float_to_fixed(float x, int scale) { return (int)round(x * (1 << scale)); }