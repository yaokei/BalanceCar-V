#include "usart1.h"

#define USART           USART1

#define TXBUFFERSIZE    (192)
#define RXBUFFERSIZE    (256)

// fill output buffers with some asciis to start with
static uint8_t tx_buffer[TXBUFFERSIZE] = "";
uint8_t rx_buffer[RXBUFFERSIZE] = "";

static int tx_counter_read = 0;
static int tx_counter_write = 0;
static int rx_counter_read = 0;
static int rx_counter_write = 0;

/**
  * @brief  Push one byte to ringbuffer of USART
  */
static int tx_push(const uint8_t *ch, uint8_t len)
{
    int ret = -1;

	// if there is free space in buffer
	if ((((tx_counter_read - tx_counter_write) - 1) + TXBUFFERSIZE) % TXBUFFERSIZE > len) {

        USART_ITConfig(USART, USART_IT_TXE, DISABLE);

		for (int i = 0; i < len; i++) {
			tx_buffer[tx_counter_write] = ch[i];
			tx_counter_write = (tx_counter_write + 1) % TXBUFFERSIZE;
		}

		ret = 0;
	}

	USART_ITConfig(USART, USART_IT_TXE, ENABLE);

	return ret;
}

/**
  * @brief  Copy from ringbuffer to USART
  */
static int tx_pop(void)
{
    int ret = -1;

	if (tx_counter_read != tx_counter_write) {
		USART_SendData(USART, tx_buffer[tx_counter_read]);
		tx_counter_read = (tx_counter_read+1) % TXBUFFERSIZE;

		ret = 0;
	}

	return ret;
}

/**
  * @brief  Copy from USART to ringbuffer
  */
static int rx_push(void)
{
    int ret = -1;

	rx_buffer[rx_counter_write] = USART_ReceiveData(USART);

	int temp = (rx_counter_write + 1) % RXBUFFERSIZE;

	if(temp != rx_counter_read) {
        // there is free space in buffer
		ret = 0;
	}

	rx_counter_write = temp;

	return ret;
}

/**
  * @brief  Pop one byte from ringbuffer of USART
  */
static int rx_pop(uint8_t *c, uint8_t len)
{
    if ((((rx_counter_write - rx_counter_read - 1) + RXBUFFERSIZE) % RXBUFFERSIZE) > len) {
        // character availability
        USART_ITConfig(USART, USART_IT_RXNE, DISABLE);

		for (int i = 0; i < len; i++) {
			c[i] = rx_buffer[rx_counter_read];
			rx_counter_read = (rx_counter_read + 1) % RXBUFFERSIZE;
		}

        USART_ITConfig(USART, USART_IT_RXNE, ENABLE);

        return 0;
    }

	return -1;
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART, USART_IT_RXNE) != RESET) {
		if(rx_push() != 0) {
			// Disable the Receive interrupt if buffer is full
			USART_ITConfig(USART, USART_IT_RXNE, DISABLE);
		}

		return;
	}

	if(USART_GetITStatus(USART, USART_IT_TXE) != RESET) {
		if(tx_pop() != 0) {
			// Disable the Transmit interrupt if buffer is empty
			USART_ITConfig(USART, USART_IT_TXE, DISABLE);
		}

		return;
	}
}

int usart1_read(uint8_t *c, uint8_t len)
{
    return rx_pop(c, len);
}

int usart1_write(const uint8_t *c, uint8_t len)
{
    return tx_push(c, len);
}

void usart1_config(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	// enable USART clock
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStruct);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	USART_Cmd(USART1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
