#include "stdint.h"
#include "usart.h"
#include "library.h"
#include "stm32f10x_dma.h"
Bluetooth_Data bluetooth;

void usart_init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //����USART1ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	

    //����PA9��ΪUSART1��Tx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA , &GPIO_InitStructure);
    //����PA10��ΪUSART1��Rx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA , &GPIO_InitStructure);

    //����USART1
    //�жϱ�������
    USART_InitStructure.USART_BaudRate = 115200;       //������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
    //����USART1ʱ��
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init(USART1, &USART_InitStructure);
    USART_ClockInit(USART1, &USART_ClockInitStruct);


    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    //ʹ��USART1


    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	                     //ʹ��DMA����
    DMA_DeInit(DMA1_Channel5);                                               

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;      //DMA�������ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)bluetooth.data_buf;                 //DMA�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                       
    DMA_InitStructure.DMA_BufferSize = 50;                                   //DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         //�����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                  //�ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;          //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                            //����������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                    //DMAͨ�� xӵ�������ȼ� 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                             //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);                             //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���

    DMA_Cmd(DMA1_Channel5, ENABLE);

    USART_Cmd(USART1, ENABLE); 

}
//void usart_dma_get(void)
//{ 
//	DMA_Cmd(DMA1_Channel5, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
// 	DMA_SetCurrDataCounter(DMA1_Channel5,50);//DMAͨ����DMA����Ĵ�С
// 	DMA_Cmd(DMA1_Channel5, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
//}
//void serialWrite(u8 c)
//{
//	USART_SendData(USART1,c);
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//}

static void uart_data_prase(uint8_t data);

void uart_databuf_get(void)  //�����жϺ���
{
    if(DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET)    	 
    {
        DMA_ClearFlag(DMA1_FLAG_TC5);
        for(int i = 0; i < 49; i++)
        {
          uart_data_prase(bluetooth.data_buf[i]);
        }
        //    usleep(1000);
    }
}
static void uart_data_prase(uint8_t data)  
{
    bluetooth.data = data;

    switch(bluetooth.state)
    {
        case 0:
            {
                if(bluetooth.data == 0xA5) 
                {
                    bluetooth.state++ ; 
                    break;
                }
                else 
                {
                    bluetooth.state = 0;
                    break;
                }
            }
        case 1:
            {
                if(bluetooth.data == 0x5A)
                {
                    bluetooth.state++;
                    break;
                }
                else 
                {
                    bluetooth.state = 0;
                    break;
                }
            }
        case 2: 
            {
                
                if(bluetooth.data == 17)
                {
                    bluetooth.len = bluetooth.data;
                    bluetooth.state++;
                    break;
                }
                else
                {
                    bluetooth.state = 0;
                    break;
                }
                
            }
        case 3: 
            {
                bluetooth.mode = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 4: 
            {
                bluetooth.ch_data[0] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 5: 
            {
                bluetooth.ch_data[1] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 6: 
            {
                bluetooth.ch_data[2] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 7: 
            {
                bluetooth.ch_data[3] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 8:
            {
                bluetooth.ch_data[4] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 9:
            {
                bluetooth.ch_data[5] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 10:
            {
                bluetooth.ch_data[6] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 11:
            {
                bluetooth.ch_data[7] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 12:
            {
                bluetooth.ch_data[8] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 13:
            {
                bluetooth.ch_data[9] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 14:
            {
                bluetooth.ch_data[10] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 15:
            {
                bluetooth.ch_data[11] = bluetooth.data;
                bluetooth.state++;
                break;
            }
        case 16:
            {
                if(0x12 == bluetooth.data)
                {
                    bluetooth.state++;
                    break;
                }
                else
                {
                    bluetooth.state = 0;
                    break;
                }
            }
        case 17:
            {
                if(0x34 == bluetooth.data)
                {
                    bluetooth.state++;
                    break;
                }
                else
                {
                    bluetooth.state = 0;
                    break;
                }
            }
        case 18:
            {
                bluetooth.ch1 = (bluetooth.ch_data[0]<<8)+bluetooth.ch_data[1];
                bluetooth.ch2 = (bluetooth.ch_data[2]<<8)+bluetooth.ch_data[3];
                bluetooth.ch3 = (bluetooth.ch_data[4]<<8)+bluetooth.ch_data[5];
                bluetooth.ch4 = (bluetooth.ch_data[6]<<8)+bluetooth.ch_data[7];
                bluetooth.ch5 = (bluetooth.ch_data[8]<<8)+bluetooth.ch_data[9];
                bluetooth.ch6 = (bluetooth.ch_data[10]<<8)+bluetooth.ch_data[11];
                bluetooth.state=0;
                break;
            }
        default:bluetooth.state=0;break;
    }
}



