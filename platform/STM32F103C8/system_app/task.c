#include "stdint.h"
#include "library.h"
#include "task.h"
#define true 1
#define COMP_FILTER 1
uint32_t slope_count = 0;

float out_left_temp = 0;
float out_right_temp = 0;
int32_t s32_left_out = 0;
int32_t s32_right_out = 0;
/******************************************************************************
		
	 
	 
******************************************************************************/
void task(void)
{
    
//	if(((_interrupt_cnt * INTERRUPT_PERIOD) %  SENSOR_PERIOD) == 0)
//	{		
		sensor_read_task(true);
//	}
	
//	if(((_interrupt_cnt * INTERRUPT_PERIOD) %  ESTIMATOR_PERIOD) == 0)
//	{		
		estimator_task(true);
//	}
	
//	if(((_interrupt_cnt * INTERRUPT_PERIOD) %  BALANCE_PERIOD) == 0)
//	{		
		balance_task(true);
//	}
    
	bluetooth_task(true);
	get_velocity();																											//get velocity
	velocity_integral(sys_time(), vel, _sensor.acc);
	
	if(slope_count > VEL_RATIO)
	{	
		slope_count = 1;
		velcontrol_task(true);
	}
	
    velocity_slope_out(slope_count, out_put.last_vel_control_out, out_put.vel_control_out);
    slope_count++;

	
	motor_task(true);
}
/******************************************************************************
		
	 
	 
******************************************************************************/
void estimator_task(uint8_t run)
{
	static uint32_t timestemp;
	uint32_t current_time = sys_time();
	uint32_t dt = current_time - timestemp;
	timestemp = current_time;
	
	if(!run)
	{
		return;
	}
	
	if(COMP_FILTER == 1)
		complementary_filter(sys_time(), _sensor.acc, _sensor.gyro, _sensor.euler);
	else
		attitude_estimator(sys_time(), _sensor.acc, _sensor.gyro, _sensor.euler);
	
	process_time.EST.time = dt;
}

/******************************************************************************
		
	 
	 
******************************************************************************/
void sensor_read_task(uint8_t run)
{
	static uint32_t timestemp;
	uint32_t current_time = sys_time();
	uint32_t dt = current_time - timestemp;
	timestemp = current_time;
	
	if(!run)
	{
		return;
	}
	
	mpu6050_get_data(_sensor.acc, _sensor.gyro);
	process_time.SENSOR.time = dt;
}

/******************************************************************************
		
	 
	 
******************************************************************************/
void balance_task(uint8_t run)
{
	if(!run)
	{
		return;
	}
	
	balance_control(sys_time(), _sensor.euler, _sensor.gyro);
}

/******************************************************************************
		
	 
	 
******************************************************************************/
void velcontrol_task(uint8_t run)
{
	
	if(!run)
	{
		return;
	}
	
	velocity_control(sys_time(), vel, _sensor.acc, vel_pid.velocity_integral);
	
	
}

/******************************************************************************
		
	 
	 
******************************************************************************/
void motor_task(uint8_t run)
{
	static uint32_t timestemp;
	uint32_t current_time = sys_time();
	uint32_t dt = current_time - timestemp;
	timestemp = current_time;
	static float left_last_out;
	static float right_last_out;
    static float pre_slope_out = 0;
	int16_t friction_limit = FMAX;
   
	
    out_put.vel_slope_out = rc_filter_process(dt, 0.8, out_put.vel_slope_out, pre_slope_out);
    pre_slope_out = out_put.vel_slope_out;
    
	out_left_temp = out_put.enguler_out - out_put.vel_slope_out + vel_pid.left_velocity;
	out_right_temp = out_put.enguler_out - out_put.vel_slope_out + vel_pid.right_velocity;
	
	out_left_temp = rc_filter_process(dt, 20, out_left_temp, left_last_out);			//20hz cut off freq rc-filter
	out_right_temp = rc_filter_process(dt, 20, out_right_temp, right_last_out);         //20hz cut off freq rc-filter
    
	left_last_out = out_left_temp;
	right_last_out = out_right_temp;
    
	s32_left_out = out_left_temp;
	s32_right_out = out_right_temp;
	
	if(s32_left_out >= 0)
		s32_left_out += friction_limit;
	else
		s32_left_out -= friction_limit;
	
	if(s32_right_out >= 0)
		s32_right_out += friction_limit;
	else
		s32_right_out -= friction_limit;
	
	s32_left_out = constrain_int32(s32_left_out, MIN_OUT_PERIOD, MAX_OUT_PERIOD);						//motor_out limit
	s32_right_out = constrain_int32(s32_right_out, MIN_OUT_PERIOD, MAX_OUT_PERIOD);					//motor_out limit
	
    
    if(!run || init_flag)
	{
		return;
	}
    
	if(s32_left_out >= 0)
	{
		TIM4->CCR1 = 0;
		TIM4->CCR2 = s32_left_out;
	}
	else if(s32_left_out < 0)
	{
		TIM4->CCR1 = -s32_left_out;
		TIM4->CCR2 = 0;
	}
	
	if(s32_right_out >= 0)
	{
		TIM4->CCR3 = s32_right_out;
		TIM4->CCR4 = 0;
	}
	else if(s32_right_out < 0)
	{
		TIM4->CCR3 = 0;
		TIM4->CCR4 = -s32_right_out;
	}
}

void get_bluetooth_data(uint32_t dt)
{
    float temp_ch3_data;
    s32 temp_ch4_data;
    static float pre_target;
    static float pre_left;
    static float pre_right;
    float get_target;
    static s32 last_ch6;
    static uint8_t start_flag = 0;
    
    if(bluetooth.ch6 < 1000 || bluetooth.ch3 < 1000)
    {
        vel_pid.target_velocity = 0;
        vel_pid.left_velocity = 0;
        vel_pid.right_velocity = 0;
        return;
    }
    
    
    get_target = -(((float)bluetooth.ch6 - 1000) / 1000.0) * 350;
    temp_ch3_data = ((float)(bluetooth.ch3 - 1500) / 500.0) * 20;
    
    if(bluetooth.ch3 > 1800 || bluetooth.ch3 < 1200)
    {
        vel_pid.left_velocity = temp_ch3_data;
        vel_pid.right_velocity = -temp_ch3_data;
    }
    else
    {
        vel_pid.left_velocity = 0;
        vel_pid.right_velocity = 0;
    }
    
    temp_ch4_data  = bluetooth.ch4;
    if(temp_ch4_data < 1200)
    {
        vel_pid.target_velocity = get_target;
    }
    else if(temp_ch4_data > 1800)
    {
        vel_pid.target_velocity = -get_target;
    }
    else
    {
        vel_pid.target_velocity = 0.0;
    }
    
    vel_pid.target_velocity = rc_filter_process(dt, 20, vel_pid.target_velocity, pre_target);
    vel_pid.left_velocity = rc_filter_process(dt, 20, vel_pid.left_velocity, pre_left);
    vel_pid.right_velocity = rc_filter_process(dt, 20, vel_pid.right_velocity, pre_right);
    
    pre_target = vel_pid.target_velocity;
    pre_left = vel_pid.left_velocity;
    pre_right = vel_pid.right_velocity;
}

void bluetooth_task(uint8_t run)
{
    if(!run)
	{
        vel_pid.target_velocity = 0;
        vel_pid.left_velocity = 0;
        vel_pid.right_velocity = 0;
		return;
	}
    
    get_bluetooth_data(sys_time());
}





	

