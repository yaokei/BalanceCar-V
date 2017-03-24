#ifndef __SYS_CLOCK_CONFIG_H
#define __SYS_CLOCK_CONFIG_H
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif
extern void set_sys_tick(void);
extern volatile uint32_t _sys_tick_count;  
extern uint32_t sys_time(void);
extern void delay_ms(uint32_t ms);
extern void delay_us(uint32_t us);
extern __irq void SysTick_Handler(void);
#ifdef __cplusplus
}
#endif
#endif




