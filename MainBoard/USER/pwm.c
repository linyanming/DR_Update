

#include "pwm.h"
#include "config.h"

u8 OrtateMotorStatus = ORTATE_STATUS_STOP;

void TIM4_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStr;
	NVIC_InitTypeDef NVIC_Str;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_BaseStr.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_BaseStr.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStr.TIM_Period = (40-1);
	TIM_BaseStr.TIM_Prescaler = 18000;
	TIM_TimeBaseInit(TIM4, &TIM_BaseStr);

	NVIC_Str.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Str.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_Str);
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

/********************************
转向电机PWM波初始化
功能：
	转向电机PWM波初始化,PWM周期为5KHz
	
参数：
	无
	
返回值：
	无
**************************************/
void Motor_Pwm_Init(void)
{
	GPIO_InitTypeDef GPIO_Str;
	TIM_TimeBaseInitTypeDef TIM_BaseStr;
	TIM_OCInitTypeDef TIM_OCStr;
	u16 pre = 0;  //分频系数

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	

	GPIO_Str.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Str.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Str);

	pre = PWM_PRESCALER;  //TIM2时钟频率为24MHz
	TIM_BaseStr.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStr.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStr.TIM_Period = PWM_PERIOD;
	TIM_BaseStr.TIM_Prescaler = pre;
	TIM_TimeBaseInit(TIM2, &TIM_BaseStr);

	TIM_OCStr.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStr.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCStr.TIM_Pulse = 0;    
	TIM_OCStr.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2, &TIM_OCStr);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OCStr.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStr.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStr.TIM_Pulse = 0;    
	TIM_OCStr.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM2, &TIM_OCStr);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OCStr.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStr.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStr.TIM_Pulse = 0;
	TIM_OCStr.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM2, &TIM_OCStr);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	DEBUGMSG("Orate_Pwm_Init pre = %d",pre);
}



/********************************
转向电机左转
功能：
	转向电机左转
	
参数：
	无
	
返回值：
	无
**************************************/
void Ortate_Motor_Left(void)
{
	TIM_SetCompare3(TIM2, ORTATE_PWM_PULSE);
	TIM_SetCompare4(TIM2, 0);
	OrtateMotorStatus = ORTATE_STATUS_LEFT;
}

/********************************
转向电机右转
功能：
	转向电机右转
	
参数：
	无
	
返回值：
	无
**************************************/
void Ortate_Motor_Right(void)
{
	TIM_SetCompare3(TIM2, 0);
	TIM_SetCompare4(TIM2, ORTATE_PWM_PULSE);
	OrtateMotorStatus = ORTATE_STATUS_RIGHT;
}

/********************************
转向电机惯性滑行（停止）
功能：
	转向电机惯性滑行（停止）
	
参数：
	无
	
返回值：
	无
**************************************/
void Ortate_Motor_Coast(void)
{
	TIM_SetCompare3(TIM2, 0);
	TIM_SetCompare4(TIM2, 0);
	OrtateMotorStatus = ORTATE_STATUS_STOP;
}

/********************************
转向电机刹车
功能：
	转向电机刹车
	
参数：
	无
	
返回值：
	无
**************************************/
void Ortate_Motor_Brate(void)
{
	TIM_SetCompare3(TIM2, PWM_PERIOD);
	TIM_SetCompare4(TIM2, PWM_PERIOD);
	OrtateMotorStatus = ORTATE_STATUS_STOP;
}

