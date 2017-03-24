#include "stm32f10x.h"
#include "stdint.h"
#include "library.h"
#include "i2c_init.h"

volatile u8 I2C_FastMode;

void i2c_delay()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	if(!I2C_FastMode)
	{
		u8 i = 15;
		while(i--);
	}
}

void i2c_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
//	 GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

int i2c_start()
{
	SDA_H;
	SCL_H;
	i2c_delay();
	if(!SDA_READ)return 0;	//SDA线为低电平则总线忙,退出
	SDA_L;
	i2c_delay();
	if(SDA_READ) return 0;	//SDA线为高电平则总线出错,退出
	SDA_L;
	i2c_delay();
	return 1;	
}

void i2c_stop()
{
	SCL_L;
	i2c_delay();
	SDA_L;
	i2c_delay();
	SCL_H;
	i2c_delay();
	SDA_H;
	i2c_delay();
}

void i2c_ask()
{
	SCL_L;
	i2c_delay();
	SDA_L;
	i2c_delay();
	SCL_H;
	i2c_delay();
	SCL_L;
	i2c_delay();
}

void i2c_noask()
{
	SCL_L;
	i2c_delay();
	SDA_H;
	i2c_delay();
	SCL_H;
	i2c_delay();
	SCL_L;
	i2c_delay();
}

int i2c_wait_ack(void) 	 //返回为:=1无ASK,=0有ASK
{
  u8 ErrTime = 0;
	SCL_L;
	i2c_delay();
	SDA_H;			
	i2c_delay();
	SCL_H;
	i2c_delay();
	while(SDA_READ)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			i2c_stop();
			return 1;
		}
	}
	SCL_L;
	i2c_delay();
	return 0;
}

void i2c_send_byte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        i2c_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        i2c_delay();
				SCL_H;
				i2c_delay();
    }
    SCL_L;
}  

//读1个字节，ack=1时，发送ACK，ack=0，发送NACK
u8 i2c_read_byte(u8 ask)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      i2c_delay();
			SCL_H;
      i2c_delay();	
      if(SDA_READ)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;

	if (ask)
		i2c_ask();
	else
		i2c_noask();  
    return ReceiveByte;
} 


// IIC写一个字节数据
u8 i2c_write_data(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	i2c_start();
	i2c_send_byte(SlaveAddress<<1);   
	if(i2c_wait_ack())
	{
		i2c_stop();
		return 1;
	}
	i2c_send_byte(REG_Address);       
	i2c_wait_ack();	
	i2c_send_byte(REG_data);
	i2c_wait_ack();   
	i2c_stop(); 
	return 0;
}

// IIC读1字节数据
u8 i2c_read_data(u8 SlaveAddress,u8 REG_Address)
{     
  u8 data;  
  
	i2c_start();
	i2c_send_byte(SlaveAddress<<1); 
	if(i2c_wait_ack())
	{
		i2c_stop();
		return 1;
	}
	i2c_send_byte(REG_Address);     
	i2c_wait_ack();
	i2c_start();
	i2c_send_byte(SlaveAddress<<1 | 0x01);
	i2c_wait_ack();
	data= i2c_read_byte(0);
	i2c_stop();
	return data;
}	

// IIC写n字节数据
u8 i2c_write_buffer(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	i2c_start();
	i2c_send_byte(SlaveAddress<<1); 
	if(i2c_wait_ack())
	{
		i2c_stop();
		return 1;
	}
	i2c_send_byte(REG_Address); 
	i2c_wait_ack();
	while(len--) 
	{
		i2c_send_byte(*buf++); 
		i2c_wait_ack();
	}
	i2c_stop();
	return 0;
}

// IIC读n字节数据
u8 i2c_read_buffer(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	i2c_start();
	i2c_send_byte(SlaveAddress<<1); 
	if(i2c_wait_ack())
	{
		i2c_stop();
		return 1;
	}
	i2c_send_byte(REG_Address); 
	i2c_wait_ack();
	
	i2c_start();
	i2c_send_byte(SlaveAddress<<1 | 0x01); 
	i2c_wait_ack();
	while(len) 
	{
		if(len == 1)
		{
			*buf = i2c_read_byte(0);
		}
		else
		{
			*buf = i2c_read_byte(1);
		}
		buf++;
		len--;
	}
	i2c_stop();
	return 0;
}



