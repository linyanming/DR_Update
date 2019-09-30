

#ifndef __LED_H__
#define __LED_H__

#include "stm32f10x_gpio.h"
#include "bit.h"
#include "config.h"

#ifdef DR_UPDATE
#define LED   PAout(9)

#define NSLEEP PBout(0)
#else
#define LED1   PBout(12)
#define LED2   PBout(13)
#define LED3   PBout(14)
#define LED4   PBout(15)

#define RGBBLUE    PAout(8)
#define RGBGREEN   PAout(9)
#define RGBRED     PAout(10)
#endif

#define BEEP  PAout(11)

void Led_Init(void);
void Led_Reset(void);

#endif
