#ifndef INC_FIXED_POINT_H_
#define INC_FIXED_POINT_H_


typedef int Q15_16;
#define SCALE_Q15_16 16

typedef int Q4_27;
#define SCALE_Q4_27 27

float fixed_to_float(int x, int scale);

int float_to_fixed(float x, int scale);

#endif /* INC_FIXED_POINT_H_ */
