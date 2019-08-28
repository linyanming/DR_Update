

#include "key.h"

u8 KeyStatus = KEYNONE;

#define NO_ACTION     0
#define PRESS_CHECK   1
#define RELEASE_CHECK 2

#define KCHECK_MAXTIMES 8

typedef struct
{
	u8 times;
	u8 k_status;
}KSCheck;
//按键检测
KSCheck KPCheck;
KSCheck KPWRCheck;

u8 KPStatus = KEYNONE;
u8 KPWRStatus = KEYNONE;
u8 KeyFlag = 0;
u8 KPWRFlag = 0;


void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_Str;
	EXTI_InitTypeDef EXTI_Str;
	NVIC_InitTypeDef NVIC_Str;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,  ENABLE);

	GPIO_Str.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Str.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_2;
//	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_Str);

	GPIO_Str.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Str.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_Str);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);

//	EXTI_Str.EXTI_Line = EXTI_Line2 | EXTI_Line4 | EXTI_Line6;
//	EXTI_Str.EXTI_Line = EXTI_Line3 | EXTI_Line4 | EXTI_Line6;
	EXTI_Str.EXTI_Line = EXTI_Line3 | EXTI_Line4;
	EXTI_Str.EXTI_LineCmd = ENABLE;
	EXTI_Str.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_Str.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_Str);
#if 0
	NVIC_Str.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Str.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_Str);
#endif
	NVIC_Str.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Str.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_Str);


	NVIC_Str.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Str.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Str);



#if 0
	NVIC_Str.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Str.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_Str);
#endif
//	EXTI_GenerateSWInterrupt();
}



void KeyShakeCheck(void)
{
	if(KPCheck.k_status == PRESS_CHECK)
	{
		if(KPCheck.times < KCHECK_MAXTIMES)
		{
			if(KEY_PAIR == 0)
			{
				KPCheck.times++;
			}
			else
			{
				KPCheck.times = 0;
				KPCheck.k_status = NO_ACTION;
			}
		}
		else
		{
			KPCheck.times = 0;
			KPCheck.k_status = NO_ACTION;
			KPStatus = KPPRESS;
		}
	}
	else if(KPCheck.k_status == RELEASE_CHECK)
	{
		if(KPCheck.times < KCHECK_MAXTIMES)
		{
			if(KEY_PAIR == 1)
			{
				KPCheck.times++;
			}
			else
			{
				KPCheck.times = 0;
				KPCheck.k_status = NO_ACTION;
			}
		}
		else
		{
			KPCheck.times = 0;
			KPCheck.k_status = NO_ACTION;
			KPStatus = KPRELEASE;
		}
	}

	if(KPWRCheck.k_status == PRESS_CHECK)
	{
		if(KPWRCheck.times < KCHECK_MAXTIMES)
		{
			if(KEY_PWR == 0)
			{
				KPWRCheck.times++;
			}
			else
			{
				KPWRCheck.times = 0;
				KPWRCheck.k_status = NO_ACTION;
			}
		}
		else
		{
			KPWRCheck.times = 0;
			KPWRCheck.k_status = NO_ACTION;
			KPWRStatus = KPWRPRESS;
		}
	}
	else if(KPWRCheck.k_status == RELEASE_CHECK)
	{
		if(KPWRCheck.times < KCHECK_MAXTIMES)
		{
			if(KEY_PWR == 1)
			{
				KPWRCheck.times++;
			}
			else
			{
				KPWRCheck.times = 0;
				KPWRCheck.k_status = NO_ACTION;
			}
		}
		else
		{
			KPWRCheck.times = 0;
			KPWRCheck.k_status = NO_ACTION;
			KPWRStatus = KPWRRELEASE;
		}
	}
}

/************************************************
按键状态确认函数

参数：
	按键标识

返回值：
	无
*************************************************/
void KeyStatusCheck(KeyEvent key)
{
	DEBUGMSG("key = %d", key);
	switch(key)
	{
		case KPPRESS:
			if(KPStatus == KEYNONE)
			{
				KPCheck.k_status = PRESS_CHECK;
				KPCheck.times = 0;
			}
			break;
		case KPRELEASE:
			if(KPStatus == KPPRESS)
			{
				KPCheck.k_status = RELEASE_CHECK;
				KPCheck.times = 0;
			}
			break;
		case KPWRPRESS:
			if(KPWRStatus == KEYNONE)
			{
				KPWRCheck.k_status = PRESS_CHECK;
				KPWRCheck.times = 0;
			}
			break;
		case KPWRRELEASE:
			if(KPWRStatus == KPWRPRESS)
			{
				KPWRCheck.k_status = RELEASE_CHECK;
				KPWRCheck.times = 0;
			}
			break;
		default:
			break;
	}

}

