#ifndef __ATTITUDE_ESTIMATOR_H
#define __ATTITUDE_ESTIMATOR_H

#include <stdint.h>

#define ESTIMATOR_PERIOD 2000

#ifdef __cplusplus
extern "C" {
#endif

extern void attitude_estimator(uint32_t t, const float *accel, float *gyro, float *euler);
extern void complementary_filter(uint32_t t, const float *accel, float *gyro, float *euler);
#ifdef __cplusplus
}
#endif
#endif

