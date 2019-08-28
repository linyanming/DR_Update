

#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "config.h"
#include "bit.h"


typedef enum
{
	KEYNONE = 0,    //无按键按下
	KPPRESS,        //pair按键按下
	KPRELEASE,      //pair按键释放
	KPWRPRESS,      //PWR按键按下
	KPWRRELEASE,    //PWR按键释放
}KeyEvent;


//按键引脚数值读取
#define KEY_PWR    PBin(4)
#define KEY_PAIR   PBin(3)

#define REED_KEY  PAin(6)

#define nF  PBin(2)

extern u8 KeyStatus;
extern u8 KeyFlag;
extern u8 KPStatus;
extern u8 KPWRStatus;
extern u8 KPWRFlag; 

void Key_Init(void);
void KeyStatusCheck(KeyEvent key);
void KeyShakeCheck(void);


#endif

