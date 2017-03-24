#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
	
typedef struct 
{
  float time;
}contain_struct_t;

typedef struct
{
	contain_struct_t TIM;
	contain_struct_t SENSOR;
	contain_struct_t EST;
} process_time_struct_t;

extern process_time_struct_t process_time;

#ifdef __cplusplus
}
#endif
#endif

