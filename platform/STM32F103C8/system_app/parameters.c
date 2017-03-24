#include "parameters.h"
#include "string.h"
#include "library.h"
#include "param.h"

uint8_t init_flag;
/**
 * @brief reset all parameters to default
 */
void params_load_defaults(void)
{
	vel_pid.target_velocity = 0.0;
	vel_pid.velocity.kp = 1.3;
	vel_pid.velocity.ki = 0.002;
	vel_pid.velocity_integral = 0.0;
		
	bal_pid.bal.kp = 15.5;
	bal_pid.bal.kd = 26;
	bal_pid.bal.ki = 0.0;
	mpu6050.acc_offset.x = -430.0;
	mpu6050.acc_offset.y = 10.0;
	mpu6050.acc_offset.z = 150.0;
	
	mpu6050.gyo_offset.x = 10.0;
	mpu6050.gyo_offset.y = -8.0;
	mpu6050.gyo_offset.z = 7.0;
	
	_sensor.gy_ratio = 81.5;
	init_flag = 1;
	
	FMAX = 15;
	kalman_Q = 0.6;
	kalman_R = 0.3;
}

void parameter_update(void)
{
	/******************************************************************************
		* velocity
  ******************************************************************************/
	param_get(param_find("VEL_TARGET"), &vel_pid.target_velocity);
	param_get(param_find("VEL_KP"), &vel_pid.velocity.kp);
	param_get(param_find("VEL_KI"), &vel_pid.velocity.ki);
	param_get(param_find("VEL_KD"), &vel_pid.velocity.kd);
	
	/******************************************************************************
		* balance
  ******************************************************************************/	
	param_get(param_find("BAL_KP"), &bal_pid.bal.kp);
	param_get(param_find("BAL_KI"), &bal_pid.bal.ki);
	param_get(param_find("BAL_KD"), &bal_pid.bal.kd);
	
	/******************************************************************************
		* sensor
  ******************************************************************************/	
	param_get(param_find("ACC_OFFSET_X"), &mpu6050.acc_offset.x);
	param_get(param_find("ACC_OFFSET_Y"), &mpu6050.acc_offset.y);
	param_get(param_find("ACC_OFFSET_Z"), &mpu6050.acc_offset.z);
	
	param_get(param_find("GRYO_OFFSET_X"), &mpu6050.gyo_offset.x);
	param_get(param_find("GRYO_OFFSET_Y"), &mpu6050.gyo_offset.y);
	param_get(param_find("GRYO_OFFSET_Z"), &mpu6050.gyo_offset.z);

	//param_get(param_find("GY_RATIO"), &_sensor.gy_ratio);
	/******************************************************************************
		* others
  ******************************************************************************/	
	param_get(param_find("LIMIT_FRICTION"), &FMAX);
	param_get(param_find("KALMAN_Q"), &kalman_Q);
	param_get(param_find("KALMAN_R"), &kalman_R);	
}

