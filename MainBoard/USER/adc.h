
#ifndef __ADC_H__
#define __ADC_H__

#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "config.h"


void ADC_Config(void);
static void DMA_ADC_Config(void);
u16* Get_ADC_Value(void);


#endif

