#ifndef __TASK_H
#define __TASK_H
#include "stdint.h"

extern uint32_t slope_count;
extern void task(void);
extern void estimator_task(uint8_t run);
extern void sensor_read_task(uint8_t run);
extern void motor_task(uint8_t run);
extern void balance_task(uint8_t run);
extern void velcontrol_task(uint8_t run);

extern void get_bluetooth_data(uint32_t dt);
extern void bluetooth_task(uint8_t run);

extern float out_left_temp;
extern float out_right_temp;
extern int32_t s32_left_out;
extern int32_t s32_right_out;

#endif
