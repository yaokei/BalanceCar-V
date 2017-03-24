/***************************************************************************
 *
 *  Copyright (c) 2017 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file attitude_estimator.c
 *
 * @author zwh <zwh@raaworks.com>
 */

#include "attitude_estimator.h"
#include "library.h"
#include <math.h>

static const float accel_gain = 0.6f; // 0.2
static const float gyro_bias_gain = 0.05f;//0.1
static float q[4];
static float q_prev[4];
static float gyro_bias[3];
static uint8_t inited = 0;

void complementary_filter(uint32_t t, const float *accel, float *gyro, float *euler)
{
	static float gyro_integral = 0;
	static int16_t count;
	float gy_temp;
	float acc_temp;
	float delta;
		
	gy_temp = _sensor.gyro[1] * _sensor.gy_ratio;
	acc_temp = _sensor.acc[0];
	euler[1] = gyro_integral;
	
	if(acc_temp >= 9.8)
		acc_temp = 9.8;
	if(acc_temp <= -9.8)
		acc_temp = -9.8;
	
	_sensor.acc_angle[1] = 3.0 * asinf((acc_temp) / 19.6)* 180 / 3.14159;
	delta = (_sensor.acc_angle[1] - euler[1]) / 3;
	
	gyro_integral += (gy_temp + delta) / 500.0;
	if(init_flag && (count < 1000))
	{
		gyro_integral = _sensor.acc_angle[1];
		count++;
	}
	else
	{
		init_flag = 0;
	}
//  MMA2=(mma+MMA_Vertical);
//  acc=MMA2;
//  gy=-1.0*T_X*gy_radio+0.4;
//  car_angle2 = gy_sum;
//  float delta=(acc*acc_radio - car_angle2)/DT;
//  gy_sum += (gy+delta)/200;
}

static uint8_t init(const float accel[])
{
	float length = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

	float z[3] = { -accel[0] / length, -accel[1] / length, -accel[2] / length };
	float x[3] = { sqrt(1.0f - z[0] * z[0]), 0.0f, -z[0] };
	float y[3] = {
		z[1] * x[2] - z[2] * x[1],
		z[2] * x[0] - z[0] * x[2],
		z[0] * x[1] - z[1] * x[0],
	};

	float R[3][3] = {
		{ x[0], x[1], x[2] },
		{ y[0], y[1], y[2] },
		{ z[0], z[1], z[2] }
	};

	float tr = R[0][0] + R[1][1] + R[2][2];
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f);
		q[0] = s * 0.5f;
		s = 0.5f / s;
		q[1] = (R[2][1] - R[1][2]) * s;
		q[2] = (R[0][2] - R[2][0]) * s;
		q[3] = (R[1][0] - R[0][1]) * s;

	}
	else {
		int i = 0;
		for (int n = 1; n < 3; n++) {
			if (R[n][n] > R[i][i]) {
				i = n;
			}
		}
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;
		float s = sqrtf(R[i][i] - R[j][j] - R[k][k] + 1.0f);
		q[i + 1] = s * 0.5f;
		s = 0.5f / s;
		q[j + 1] = (R[i][j] + R[j][i]) * s;
		q[k + 1] = (R[k][i] + R[i][k]) * s;
		q[0] = (R[k][j] - R[j][k]) * s;
	}

	length = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] /= length;
	q[1] /= length;
	q[2] /= length;
	q[3] /= length;

	gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;

	uint8_t ret = 0;

	if (isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3])
		&& length > 0.95f && length < 1.05f) {

		ret = 1;
	}

	return ret;
}

void attitude_estimator(uint32_t t, const float *accel, float *gyro, float *euler)
{
	static uint64_t timestamp = 0;

	float dt = timestamp > 0 ? (t - timestamp) * 1e-6f : 1e-6f;
	dt = dt < 0.0001f ? 0.0001f : dt > 0.02f ? 0.02f : dt;
	timestamp = t;

	if (!inited) {
		inited = init(accel);
		return;
	}

	float scale;

	scale = 1.0f / sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	float a[3] = { accel[0] * scale, accel[1] * scale, accel[2] * scale };

	float b[3] = {
		2.0f * (q[1] * q[3] - q[0] * q[2]),
		2.0f * (q[2] * q[3] + q[0] * q[1]),
		q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
	};

	float c[3] = {
		b[1] * a[2] - b[2] * a[1],
		b[2] * a[0] - b[0] * a[2],
		b[0] * a[1] - b[1] * a[0],
	};

	c[0] *= accel_gain;
	c[1] *= accel_gain;
	c[2] *= accel_gain;

	gyro_bias[0] += c[0] * gyro_bias_gain * dt;
	gyro_bias[1] += c[1] * gyro_bias_gain * dt;
	gyro_bias[2] += c[2] * gyro_bias_gain * dt;

	gyro_bias[0] = gyro_bias[0] < -0.05f ? -0.05f : gyro_bias[0] > 0.05f ? 0.05f : gyro_bias[0];
	gyro_bias[1] = gyro_bias[1] < -0.05f ? -0.05f : gyro_bias[1] > 0.05f ? 0.05f : gyro_bias[1];
	gyro_bias[2] = gyro_bias[2] < -0.05f ? -0.05f : gyro_bias[2] > 0.05f ? 0.05f : gyro_bias[2];

	gyro[0] += gyro_bias[0];
	gyro[1] += gyro_bias[1];
	gyro[2] += gyro_bias[2];
	
	c[0] += gyro[0];
	c[1] += gyro[1];
	c[2] += gyro[2];
	
//	c[0] += gyro[0] + gyro_bias[0];
//	c[1] += gyro[1] + gyro_bias[1];
//	c[2] += gyro[2] + gyro_bias[2];

	float q_prev[4] = { q[0], q[1], q[2], q[3] };

	q[0] += (c[0] * -q_prev[1] + c[1] * -q_prev[2] + c[2] * -q_prev[3]) * 0.5f * dt;
	q[1] += (c[0] * q_prev[0] + c[1] * -q_prev[3] + c[2] * q_prev[2]) * 0.5f * dt;
	q[2] += (c[0] * q_prev[3] + c[1] * q_prev[0] + c[2] * -q_prev[1]) * 0.5f * dt;
	q[3] += (c[0] * -q_prev[2] + c[1] * q_prev[1] + c[2] * q_prev[0]) * 0.5f * dt;

	scale = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= scale;
	q[1] *= scale;
	q[2] *= scale;
	q[3] *= scale;

	if (!isfinite(q[0]) || !isfinite(q[1]) || !isfinite(q[2]) || !isfinite(q[3])) {
		q[0] = q_prev[0];
		q[1] = q_prev[1];
		q[2] = q_prev[2];
		q[3] = q_prev[3];
		gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
	}

    euler[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 180.0f / 3.14159f;
    euler[1] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180.0f / 3.14159f;
    euler[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 180.0f / 3.14159f;
}



