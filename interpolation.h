#ifndef INC_INTERPOLATION_H_
#define INC_INTERPOLATION_H_

struct lerp {
	float x_0;
	float step;
	int t_n;
};
typedef struct lerp Lerp;

void lerp_init(Lerp *lerp, float x_0, float x_n, int t_n);
float lerp_interpolate(Lerp *lerp, int t);

#endif /* INC_INTERPOLATION_H_ */