

#include "led.h"

/********************************
LED初始化函数
功能：
	初始化LED相关的IO口
	
参数：
	无
	
返回值：
	无
**************************************/

void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_Str;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_Str.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Str.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_Str);

	GPIO_Str.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Str.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Str);

//	Led_Reset();
	DEBUGMSG("LED init");
}

/********************************
LED重置函数
功能：
	将LED灯全部熄灭
	
参数：
	无
	
返回值：
	无
**************************************/
void Led_Reset(void)
{
	LED1 = 0;
	LED2 = 0;
	LED3 = 0;
	LED4 = 0;
	RGBBLUE = 0;
	RGBGREEN = 0;
	RGBRED = 0;
}

