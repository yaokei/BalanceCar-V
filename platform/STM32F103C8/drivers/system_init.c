#include "stm32f10x.h"
#include "system_init.h"
#include "library.h"
/******************************************************************************
		
	 
	 
******************************************************************************/
void system_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	set_sys_tick();
	rgb_led_init();
	i2c_init();
	mpu6050_init();
	delay_ms(100);
	mpu6050_init();
	delay_ms(100);
	pwm_init();
	tim2_init(2000);
	encoder_config();
////	usart_init();
}



