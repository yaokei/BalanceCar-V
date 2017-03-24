#include "stdint.h"
#include "library.h"
#include "ComFilter.h"

float kalman_Q;
float kalman_R;
	
/******************************************************************************
		
	 
	 
******************************************************************************/
float rc_filter(float dt, float cuoff_freq_hz)
{
    // calculate alpha
    dt = dt / 1000000;
		if(cuoff_freq_hz <= 0.0f || dt <= 0.0f)
		{
			return 0.001;
		}
		
    float rc = 1.0 / (M_2PI * cuoff_freq_hz);
    return dt / (dt + rc);
}

float rc_filter_process(float dt, float cuoff_freq_hz, float now_data, float pre_data)
{
    // calculate alpha
        dt = dt / 1000000;
		if(cuoff_freq_hz <= 0.0f || dt <= 0.0f)
		{
			return pre_data;
		}
		
    float rc = 1.0 / (M_2PI * cuoff_freq_hz);
		
    return (pre_data + (now_data - pre_data) * (dt / (dt + rc)));
}
/******************************************************************************
		
	 
	 
******************************************************************************/
#define NUM 5
float sild_filter_velocity(float input)
{
	static float buffer[NUM];
	float sum = 0.000;
	int i;
	
	for(i = NUM-1;i > 0;i--)
	{
	 *(buffer+i)=*(buffer+i-1);
	}
	 
	*buffer = input;
	
	for(i = NUM;i > 0;i--)
	{
		sum += buffer[NUM - 1];
	}
	
	sum = sum / (float)NUM;
	return sum;
}

/*      
        Q:过程噪声， Q增大， 动态响应变快，收敛稳定性变坏
        R:测量噪声， R增大， 动态响应变慢，收敛稳定性变好
	gg_sumx= KalmanFilter1(g_sumx,2,2); 
*/

float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
        float R = MeasureNoise_R;
        float Q = ProcessNiose_Q;

        static float x_last;

        float x_mid = x_last;
        float x_now;

        static float p_last;

        float p_mid ;
        float p_now;
        float kg;      

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
        kg=p_mid/(p_mid+R); //kg?kalman filter,R???
        x_now=x_mid+kg*(ResrcData-x_mid);//???????
        p_now=(1-kg)*p_mid;//??????covariance      
        p_last = p_now; //??covariance?
        x_last = x_now; //???????
        return x_now;               
}

/******************************************************************************
		
	 
	 
******************************************************************************/
inline float constrain_float(float v, float min, float max)
{
	return v < min ? min : v > max ? max : v;
}

/******************************************************************************
		
	 
	 
******************************************************************************/
inline uint32_t constrain_uint32(uint32_t v, uint32_t min, uint32_t max)
{
	return v < min ? min : v > max ? max : v;
}

/******************************************************************************
		
	 
	 
******************************************************************************/
inline int32_t constrain_int32(int32_t v, int32_t min, int32_t max)
{
	return v < min ? min : v > max ? max : v;
}

