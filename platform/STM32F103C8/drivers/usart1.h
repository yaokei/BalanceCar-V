
#pragma once

#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern void usart1_config(void);

extern int usart1_read(uint8_t *c, uint8_t len);
extern int usart1_write(const uint8_t *c, uint8_t len);

#ifdef __cplusplus
}
#endif
