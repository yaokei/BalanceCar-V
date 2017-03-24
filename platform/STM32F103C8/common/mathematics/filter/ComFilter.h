#ifndef __COMFILTER_H
#define __COMFILTER_H
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif
	
#define M_PI_FILT 3.14159f
#define M_2PI    (M_PI_FILT * 2.0)

extern float kalman_Q;
extern float kalman_R;
	
extern float rc_filter(float dt, float cuoff_freq_hz);
extern inline float constrain_float(float v, float min, float max);
extern inline uint32_t constrain_uint32(uint32_t v, uint32_t min, uint32_t max);
extern inline int32_t constrain_int32(int32_t v, int32_t min, int32_t max);
extern float sild_filter_velocity(float input);
extern float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
extern float rc_filter_process(float dt, float cuoff_freq_hz, float now_data, float pre_data);
	
#ifdef __cplusplus
}
#endif

#endif

