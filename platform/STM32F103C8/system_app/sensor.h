#ifndef __SENSOR_H
#define __SENSOR_H

#include "stdint.h"

#define SENSOR_PERIOD	2000		// us

typedef struct
{
} est_struct_t;

typedef struct
{
	float acc[3];			// m/ss
	float gyro[3];		// degree/s
	float euler[3];
	float acc_angle[3];
	float gy_ratio;
	est_struct_t engine;
} sensor_struct_t;

extern sensor_struct_t _sensor;

extern void sensor_init(void);


















#endif
