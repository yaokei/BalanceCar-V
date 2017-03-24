#ifndef __I2C_INIT_H
#define	__I2C_INIT_H
#include "stdint.h"
#include "stm32f10x_gpio.h"

/***************I2C GPIO¶¨Òå******************/
#define ANO_GPIO_I2C	GPIOB
#define I2C_PIN_SCL		GPIO_Pin_10
#define I2C_PIN_SDA		GPIO_Pin_11
//#define I2C_PIN_SCL		GPIO_Pin_7
//#define I2C_PIN_SDA		GPIO_Pin_6
#define ANO_RCC_I2C		RCC_AHB1Periph_GPIOB
/*********************************************/

#define SCL_H         ANO_GPIO_I2C->BSRR = I2C_PIN_SCL
#define SCL_L         ANO_GPIO_I2C->BRR = I2C_PIN_SCL
#define SDA_H         ANO_GPIO_I2C->BSRR = I2C_PIN_SDA
#define SDA_L         ANO_GPIO_I2C->BRR = I2C_PIN_SDA
#define SCL_READ      ANO_GPIO_I2C->IDR  & I2C_PIN_SCL
#define SDA_READ      ANO_GPIO_I2C->IDR  & I2C_PIN_SDA

#ifdef __cplusplus
extern "C"
{
#endif
extern volatile u8 I2C_FastMode;
	
extern void i2c_delay(void);
extern void i2c_init(void);
extern int i2c_start(void);
extern void i2c_stop(void);
extern void i2c_ask(void);
extern void i2c_noask(void);
extern int i2c_wait_ack(void);
extern u8 i2c_read_byte(u8 ask);
extern u8 i2c_write_data(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
extern u8 i2c_read_data(u8 SlaveAddress,u8 REG_Address);
extern u8 i2c_write_buffer(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
extern u8 i2c_read_buffer(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
#ifdef __cplusplus
}
#endif
#endif

