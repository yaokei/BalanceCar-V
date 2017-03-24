#include "encoder.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "library.h"

#define ENCODER1_PORTA             GPIOC
#define ENCODER1_PORTB             GPIOB
#define ENCODER1_PINA              GPIO_Pin_14
#define ENCODER1_PINB              GPIO_Pin_2

#define ENCODER2_PORTA             GPIOC
#define ENCODER2_PORTB             GPIOA
#define ENCODER2_PINA              GPIO_Pin_13
#define ENCODER2_PINB              GPIO_Pin_0

#define scale											 0.2f

u32 left_direct;
u32 right_direct;
u32 pre_left_direct;
u32 pre_right_direct;

float encoder1_speed;
float encoder2_speed;


void encoder_config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;   
  EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);     // 使能复用功能时钟
  
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB , &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOC , &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOC , &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB , &GPIO_InitStructure);
  
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource2);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

  EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line2;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;              
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;    
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;           
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;              
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;           
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                
  NVIC_Init(&NVIC_InitStructure);
}

void EXTI2_IRQHandler(void)
{	 
  u32 current_time = sys_time();
  static u32 begin_time = 0;
  static float pre_vel_1;

  if(EXTI_GetITStatus(EXTI_Line2) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line2);
    if(GPIO_ReadInputDataBit(ENCODER1_PORTA, ENCODER1_PINA) == 1)
    {
      encoder1_speed = (-1000000.0f/(float)(current_time - begin_time)) * scale; 
      left_direct  = 1;
    }
    else
    {
      encoder1_speed = (1000000.0f/(float)(current_time - begin_time)) * scale;
      left_direct  = 0;
    }
    
    if(left_direct != pre_left_direct)
    {
      encoder1_speed = 0;
    }
    
    pre_vel_1 = encoder1_speed;
    pre_left_direct = left_direct;
    
    begin_time = current_time;
  }
}
void EXTI0_IRQHandler(void)
{	 
  u32 current_time = sys_time();
  static u32 begin_time = 0;
  static float pre_vel_2;
    
  if(EXTI_GetITStatus(EXTI_Line0) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line0);
    if(GPIO_ReadInputDataBit(ENCODER2_PORTA, ENCODER2_PINA) == 1)
    {
      encoder2_speed = (1000000.0f/(float)(current_time - begin_time)) * scale;
      right_direct = 1;
    }
    else 
    {
      encoder2_speed = (-1000000.0f/(float)(current_time - begin_time)) * scale;
      right_direct = 0;
    }
    
    if(right_direct != pre_right_direct)
    {
      encoder2_speed = 0;
    }
    
    pre_vel_2 = encoder1_speed;
    pre_right_direct = right_direct;
    
    begin_time = current_time;
  }
}

