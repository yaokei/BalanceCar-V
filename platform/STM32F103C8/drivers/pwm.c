#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stdint.h"
#include "pwm.h"

void pwm_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //PWM--->1KHZ
  TIM_TimeBaseStructure.TIM_Period = MAX_OUT_PERIOD;   
  TIM_TimeBaseStructure.TIM_Prescaler=36 - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;   	 
  //PB6
  TIM_OCInitStructure.TIM_Pulse=0;    
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  //PB7
  TIM_OCInitStructure.TIM_Pulse=0;    
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  //PB8     
  TIM_OCInitStructure.TIM_Pulse=0;     
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  //PB9      
  TIM_OCInitStructure.TIM_Pulse=0;    
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	
  TIM_Cmd(TIM4, ENABLE);
}
