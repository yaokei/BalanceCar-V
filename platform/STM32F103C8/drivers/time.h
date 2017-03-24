#ifndef __TIME_H
#define __TIME_H

#include "stdint.h"

#define INTERRUPT_PERIOD 2000

extern volatile uint32_t _interrupt_cnt;
extern void tim2_init(uint16_t period_num);
extern __irq void TIM2_IRQHandler(void);

#endif

