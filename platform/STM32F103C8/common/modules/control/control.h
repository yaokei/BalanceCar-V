#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdint.h"

#define VEL_RATIO 20
#define BALANCE_PERIOD 2000
#define VELOCITY_PERIOD BALANCE_PERIOD * VEL_RATIO

typedef struct
{
	uint8_t flag;
}temp_t;

typedef struct
{
	float kp;
	float ki;
	float kd;
}pid_struct_t;

typedef struct
{
	float enguler_out;
	float vel_control_out;
	float last_vel_control_out;
	float vel_slope_out;
	temp_t temp_t;
}out_struct_t;

typedef struct
{
	float target_velocity;
	pid_struct_t velocity;
	float velocity_integral;
	float vel_error;
    
    float left_velocity;
    float right_velocity;
}velocity_struct_t;

typedef struct
{
	pid_struct_t bal;
}balance_struct_t;

extern float FMAX;
extern float vel;
extern out_struct_t out_put;
extern velocity_struct_t vel_pid;
extern balance_struct_t bal_pid;

extern void get_velocity(void);
extern void balance_control(uint32_t t, float *euler, float *gyro);
extern void velocity_control(uint32_t t, float current_velocity, const float *acc, float integral);
extern void velocity_integral(uint32_t t, float current_velocity, const float *acc);
extern void velocity_slope_out(float slope_count, float old_vel_out, float new_vel_out);


#endif
