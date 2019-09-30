

#include "adc.h"


#define ADC_DR_Address (ADC1_BASE + 0x4C)

__IO u16 ADC_Value[3];   //ADC转化后的值

/************************************************
ADC初始化函数

参数：
	无

返回值：
	无
*************************************************/

void ADC_Config(void)
{
	GPIO_InitTypeDef GPIO_Str;
	ADC_InitTypeDef ADC_Str;

	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_Str.GPIO_Mode = GPIO_Mode_AIN;
#ifdef DR_UPDATE
	GPIO_Str.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5 | GPIO_Pin_6;
#else
	GPIO_Str.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_5;
#endif
//	GPIO_Str.GPIO_Pin = GPIO_Pin_4;

	GPIO_Init(GPIOA, &GPIO_Str);

	DMA_ADC_Config();
	
	ADC_DeInit(ADC1);
	ADC_Str.ADC_ContinuousConvMode = ENABLE;
	ADC_Str.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_Str.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_Str.ADC_Mode = ADC_Mode_Independent;
	ADC_Str.ADC_NbrOfChannel = 3;
//	ADC_Str.ADC_NbrOfChannel = 1;
	ADC_Str.ADC_ScanConvMode = ENABLE;
//	ADC_Str.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_Str);

#ifdef DR_UPDATE
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_239Cycles5);
#else
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_239Cycles5);
#endif

	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	DEBUGMSG("ADC Init");
}

/************************************************
针对ADC的DMA初始化函数
参数：
	无

返回值：
	无
*************************************************/

static void DMA_ADC_Config(void)
{
	DMA_InitTypeDef DMA_Str;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel1);

	DMA_Str.DMA_BufferSize = 3;               //数据数目    : 3
	DMA_Str.DMA_DIR = DMA_DIR_PeripheralSRC;  //传输方向: 外设到DMA
	DMA_Str.DMA_M2M = DMA_M2M_Disable;        
	DMA_Str.DMA_MemoryBaseAddr = (u32)&ADC_Value[0];  //内存储存地址
	DMA_Str.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;   //储存大小：半字 即：两个字节
	DMA_Str.DMA_MemoryInc = DMA_MemoryInc_Enable;  //储存地址递增
	DMA_Str.DMA_Mode = DMA_Mode_Circular;   //循环模式
	DMA_Str.DMA_PeripheralBaseAddr = (u32)ADC_DR_Address;  //外设地址
	DMA_Str.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //外设数据大小：半字 即：两个字节
	DMA_Str.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Str.DMA_Priority = DMA_Priority_High;  //优先级
	DMA_Init(DMA1_Channel1, &DMA_Str);

	DMA_Cmd(DMA1_Channel1, ENABLE);
}


/************************************************
ADC采样数值获取函数
参数：
	无

返回值：
	ADC转换值
*************************************************/
u16* Get_ADC_Value(void)
{
//	while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
//	DMA_ClearFlag(DMA1_FLAG_TC1);
//	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
//	ADC_Value[1] = ADC_GetConversionValue(ADC1);
//	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
//	printf("adc_value = %d\r\n",ADC_Value[1]);
//	DEBUGMSG("GET adc value = %d", ADC_Value);
//	DEBUGMSG("GET adc value[0] = %d value[1] = %d value[2] = %d",ADC_Value[0],ADC_Value[1],ADC_Value[2]);
	return ADC_Value;
}

