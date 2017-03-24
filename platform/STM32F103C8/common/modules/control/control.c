#include "stdint.h"
#include "control.h"
#include <math.h>
#include "library.h"

#define INTGE_LIMIT 600
u32 wrong_cont;
float FMAX;
out_struct_t out_put;
velocity_struct_t vel_pid;
balance_struct_t bal_pid;

float vel;

float lala;

void balance_control(uint32_t t, float *euler, float *gyro)
{
	float damp_out = (bal_pid.bal.kd * gyro[1]);
	float acc_temp = _sensor.acc[0];
	if((fabs(gyro[1]) > 35) && (fabs(gyro[1]) < 85))
	{
		damp_out *= (1 + ((fabs(gyro[1]) - 35) / 35)); 
	}
	
	out_put.enguler_out = bal_pid.bal.kp * euler[1] + damp_out + bal_pid.bal.ki * acc_temp;
	
	if(acc_temp > 9.8)
		acc_temp = 9.8;
	if(acc_temp < -9.8)
		acc_temp = -9.8;
	_sensor.acc_angle[0] = 3.0 * asinf((acc_temp) / 19.6)* 180 / 3.14159;
	
}

void get_velocity(void)
{
	float current_velocity;
    static uint32_t timestemp;
    uint32_t current_time = sys_time();
    static float pre_vel;
    static float vel_arr[20] = {0};
    
    float dt = current_time - timestemp;
    timestemp = current_time;
    
	current_velocity = (-(encoder1_speed + encoder2_speed) / 2.0f) + _sensor.gyro[1] * 18.0;
    encoder1_speed = 0;
	encoder2_speed = 0;
    
    vel = current_velocity;
//    vel = 0;
//    for(int i= 0;i < 19;i++)
//    {
//      vel_arr[i] = vel_arr[i+1]; 
//	  vel += vel_arr[i];
//    }
//    
//	vel_arr[19]=current_velocity;
//	vel += vel_arr[19];
//   
//    vel = vel / 20.0;
//    
//    if(vel > 300 || vel < -300)
//    {
//        wrong_cont++;
//        vel = pre_vel;
//    }
//    
//    pre_vel = vel;
    vel = rc_filter_process(sys_time(), 10, current_velocity, pre_vel);
    pre_vel = vel;
	//vel = KalmanFilter(current_velocity, kalman_Q, kalman_R);
    
}

void velocity_integral(uint32_t t, float current_velocity, const float *acc)
{
	static uint32_t timestemp;
	static float pre_error[20]={0};
	float dt = t - timestemp;
	float integral_temp = 0.0f;
    float vel_error;
	timestemp = t;
	
	vel_error = vel_pid.target_velocity - current_velocity;
    
//    if(fabs(vel_error) < 30)
//    {
        for(int i= 0;i < 19;i++)
       {
        pre_error[i] = pre_error[i+1]; 
          integral_temp += pre_error[i];
       }
        pre_error[19]=vel_error;
        integral_temp += pre_error[19];
//   }
	
	vel_pid.velocity_integral  =  0.00001 * integral_temp * vel_pid.velocity.ki * dt / 1000.0f;								//t ->> ms
	//vel_pid.velocity_integral += vel_pid.vel_error * vel_pid.velocity.ki * dt / 1000.0f; 	//t ->> ms
	
	vel_pid.velocity_integral = constrain_float(vel_pid.velocity_integral, -60, 60);						  	//integral limit
}

void velocity_control(uint32_t t, float current_velocity, const float *acc, float integral)
{
	static uint32_t timestemp;
	static u8 count = 0;
	float dt = t - timestemp;
	float pout_temp;
	float iout_temp;
	float dout_temp;
    static float pre_vel_error;
    float error_delta;
    
	timestemp = t;

	vel_pid.vel_error = vel_pid.target_velocity - current_velocity;

    error_delta = vel_pid.vel_error - pre_vel_error;
    
//    if(error_delta > 50)
//        vel_pid.vel_error = pre_vel_error + 30;
//    
//    if(error_delta < -50)
//        vel_pid.vel_error = pre_vel_error - 30;
    
    pre_vel_error = vel_pid.vel_error;
    
	pout_temp = vel_pid.vel_error * vel_pid.velocity.kp;
	iout_temp = integral;
	out_put.last_vel_control_out = out_put.vel_control_out;
	out_put.vel_control_out = pout_temp + iout_temp;
    
	
	//out_put.vel_control_out = rc_filter_process(dt, 500, out_put.vel_control_out, out_put.last_vel_control_out);
	
	out_put.vel_control_out = constrain_float(out_put.vel_control_out, -400.0, 400.0);
}

void velocity_slope_out(float slope_count, float old_vel_out, float new_vel_out)
{
	out_put.vel_slope_out = ((new_vel_out - old_vel_out) * slope_count / VEL_RATIO) + old_vel_out;
}

	


