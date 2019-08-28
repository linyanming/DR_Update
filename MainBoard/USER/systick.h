#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "misc.h"
#include "config.h"

//void Systick_Init(u8 systick);
//void delay_us(u32 nus);   //24位计数器nus最大值2^24/tick_us(798915us)
//void delay_nms(u32 nms);   //24位计数器nms最大值2^24/tick_ms(798ms)
void delay_ms(u32 ms);

void TimingDelay_Decrement(void);


void DelayMs(__IO u32 ms);


#endif



