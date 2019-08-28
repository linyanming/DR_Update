

#include "systick.h"

//u32 tick_us;  //一个微秒所用的计数器值
//u16 tick_ms;  //一个毫秒所用的计数器值


__IO u32 TimingDelay = 0;


void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}


/****************************************
systick定时器初始化函数

参数：
	systick:当前mcu的时钟频率 单位MHz 举例：48MHz填48即可

返回值：无

*********************************************/
/*
void Systick_Init(u8 systick)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	tick_us = systick/8;         
	tick_ms = (u16)tick_us*1000;
}
*/
/*********************************
微秒延时函数

参数：
	nus：延时多少微秒 最大值798915
	
返回值：无

********************************************/
/*
void delay_us(u32 nus)   //24位计数器nus最大值2^24/tick_us(798915us)
{
	u32 temp;
	SysTick->LOAD = nus*tick_us;
	SysTick->VAL = 0x00;
	SysTick->CTRL |= 0x01;
	do
	{
		temp = SysTick->CTRL;		
	}
	while ((temp&0x01) && !(temp&(1 << 16)));
	SysTick->CTRL &= ~0x01;
	SysTick->VAL = 0x00;
}
*/
/*********************************
毫秒延时函数

参数：
	nms：延时多少毫秒 最大值798ms
	
返回值：无

********************************************/
/*
void delay_nms(u32 nms)   //24位计数器nms最大值2^24/tick_ms(798ms)
{
	u32 temp;
	SysTick->LOAD = nms*tick_ms;
	SysTick->VAL = 0x00;
	SysTick->CTRL |= 0x01;
	do
	{
		temp = SysTick->CTRL;		
	}
	while ((temp&0x01) && !(temp&(1 << 16)));
	SysTick->CTRL &= ~0x01;
	SysTick->VAL = 0x00;
}
*/
/*********************************
毫秒延时函数

参数：
	ms：延时多少毫秒 无最大值限定
	
返回值：无

********************************************/
void delay_ms(__IO u32 ms)
{
	TimingDelay = ms;
	while(TimingDelay != 0);
}

void DelayMs(__IO u32 ms)
{
	u32 i = 8000 * ms;
	while(i--);
}
/*
void delay_ms(u32 ms)
{
	u8 repeat = ms/540;
	u16 remain = ms%540;
	while(repeat)
	{
		delay_nms(540);
		repeat--;
	}
	if(remain)
		delay_nms(remain);
}
*/

