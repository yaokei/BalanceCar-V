/********************************************************************************
 *                         R&A Works
 *
 * @file  : main.cpp
 * @brief : mcu initialization
 *
 * @author:
 
 *
 * www.raaworks.com
 ********************************************************************************/

#include "stm32f10x.h"
#include "stdint.h"
#include "main.hpp"
#include "library.h"
#include "platform.h"

process_time_struct_t process_time;								
#define BLUETOOTH 0	

int main()
{
	system_init();
	params_load_defaults();
	hal_config();

	while (1)
	{
		uart_databuf_get();
		//rgb_test(true);
		//parameter_update();
		loop();
	}
}




