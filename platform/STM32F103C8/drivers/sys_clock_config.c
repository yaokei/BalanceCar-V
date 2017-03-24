#include "stm32f10x.h"
#include "sys_clock_config.h"

const uint32_t _sys_clock = 72;              // System clock @MHz
const uint32_t _sys_tick_period = 100000;    // SysTick period @us
volatile uint32_t _sys_tick_count = 0;       // SysTick interrupt count

/******************************************************************************
		
	 
	 
******************************************************************************/
uint32_t sys_time(void)
{
    uint32_t t = ((float)_sys_tick_count + 1.0f - (float)SysTick->VAL / (float)SysTick->LOAD) * (float)_sys_tick_period;

    return t;
}

/******************************************************************************
		
	 
	 
******************************************************************************/
void set_sys_tick(void)
{
  uint32_t ticks;
  ticks = _sys_tick_period * _sys_clock;
  SysTick_Config(ticks);
  NVIC_SetPriority(SysTick_IRQn, 2); 		//high bit use for Preemptive priority, low bit use for SubPriority
}

/******************************************************************************
		
	 
	 
******************************************************************************/
void delay_us(uint32_t us)
{
	uint32_t time_stamp;
	time_stamp = sys_time();
	while (sys_time() < (time_stamp + us));
}

/******************************************************************************
		
	 
	 
******************************************************************************/
void delay_ms(uint32_t ms)
{
	delay_us(ms * 1000);
}

__irq void SysTick_Handler(void)
{
    _sys_tick_count ++;
}
