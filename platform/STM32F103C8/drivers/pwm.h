#ifndef __PWM_H
#define __PWM_H
#include "stdint.h"

extern void pwm_init(void);

#define MAX_OUT_PERIOD (200 - 1)
#define MIN_OUT_PERIOD -(200 - 1)

#endif
