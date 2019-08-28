
#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#define PWM_FRE 5000  //5Khz 频率

#define PWM_PERIOD (4800 - 1)  //PWM计数值
#define PRESCALER (PWM_FRE * (PWM_PERIOD + 1))  //时钟24Mhz

#define PWM_PRESCALER ((u16)(SystemCoreClock / PRESCALER) - 1)  //分频系数

#define ORTATE_PWM_PRE 80  //转向电机PWM占空比
#define ORTATE_PWM_PULSE ((ORTATE_PWM_PRE * (PWM_PERIOD + 1) / 100)) //转向电机占空比值

#define ORTATE_STATUS_LEFT 3
#define ORTATE_STATUS_RIGHT 2
#define ORTATE_STATUS_STOP 1

#define SPEED_BASE 550
#define SPEED_OFFSET 1300
#define SPEED_SWITCH_BASE 50

#define SPEED0 0
#define SPEED1 1
#define SPEED2 2
#define SPEED3 3
#define SPEED4 4
#define SPEED5 5
#define SPEED6 6
#define SPEED7 7

#define SPEED0_VAL 0
#define SPEED1_VAL SPEED_OFFSET
#define SPEED2_VAL (SPEED_OFFSET+SPEED_BASE)
#define SPEED3_VAL (SPEED_OFFSET+SPEED_BASE*2)
#define SPEED4_VAL (SPEED_OFFSET+SPEED_BASE*3)
#define SPEED5_VAL (SPEED_OFFSET+SPEED_BASE*4)
#define SPEED6_VAL (SPEED_OFFSET+SPEED_BASE*5)
#define SPEED7_VAL (SPEED_OFFSET+SPEED_BASE*6)

#if 0
#define SPEED0_VAL (SPEED0*SPEED_BASE)
#define SPEED1_VAL (SPEED1*SPEED_BASE)
#define SPEED2_VAL (SPEED2*SPEED_BASE)
#define SPEED3_VAL (SPEED3*SPEED_BASE)
#define SPEED4_VAL (SPEED4*SPEED_BASE)
#define SPEED5_VAL (SPEED5*SPEED_BASE)
#define SPEED6_VAL (SPEED6*SPEED_BASE)
#define SPEED7_VAL (SPEED7*SPEED_BASE)
#endif

extern u8 OrtateMotorStatus;


void Motor_Pwm_Init(void);
void Ortate_Motor_Left(void);
void Ortate_Motor_Right(void);
void Ortate_Motor_Coast(void);
void Ortate_Motor_Brate(void);

void TIM4_Config(void);

#endif

