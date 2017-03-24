#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    unsigned char data;
	  unsigned char data_buf[50];
		unsigned char data_buf_index;
    int len;
    unsigned char mode;  
    unsigned char state;
    
    unsigned char ch_data[12];
    
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int ch5;
    int ch6;
}Bluetooth_Data;

extern Bluetooth_Data bluetooth;
void uart_databuf_get(void);
extern void usart_init(void);
void serialWrite(u8 c);
//extern void uart_databuf_get();
#ifdef __cplusplus
}
#endif
#endif
