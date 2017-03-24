#include "rgb_led.h"
#include "library.h"

void rgb_led_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	       
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void red_set(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
  GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

void green_set(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_11);
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

void blue_set(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_15);
  GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

void rgb_control(void)
{
	

}

void rgb_test(uint8_t run)
{
	red_set();
	delay_ms(1000);
	green_set();
	delay_ms(1000);
	blue_set();
	delay_ms(1000);
}
