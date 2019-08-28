
#include "usart.h"



/*********************************
功能：
	串口1初始化

参数：
	baud：波特率
返回值：无

**********************************/

void Usart1_Init(u32 baud)
{
	GPIO_InitTypeDef GPIO_Str;
	USART_InitTypeDef USART_Str;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	GPIO_Str.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Str.GPIO_Pin = GPIO_Pin_6;
	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_Str);

	GPIO_Str.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Str.GPIO_Pin = GPIO_Pin_7;
//	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_Str);

	

	USART_Str.USART_BaudRate = baud;
	USART_Str.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Str.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Str.USART_Parity = USART_Parity_No;
	USART_Str.USART_StopBits = USART_StopBits_1;
	USART_Str.USART_WordLength = USART_WordLength_8b;
	USART_Str.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_Str);

	USART_Cmd(USART1, ENABLE);
}


