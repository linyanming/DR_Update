
#include "main_control.h"
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "led.h"
#include "systick.h"
#include "pwm.h"
#include "key.h"
#include "adc.h"
#include "iwdg.h"
#include "can.h"

__IO u8 pwr_status;  //停机开机管理变量
__IO u16 pwr_time;  //关机时间计数
u8 ConnStatus; //连接状态
u8 DeviceMode; //控制模式
u8 MoveMotorStatus;  //前进电机状态
//u8 OrateMotorStatus; //转向电机状态
__IO u16 MotorMoveTime;   //前进时间
__IO u16 OrateMoveTime;  //转向电机时间
u8 Speed;      //前进电机速度
u8 Speed_temp; //速度切换缓存变量
u16 target_speed; //目标速度
u16 now_speed;    //当前速度
__IO u32 BeepIndTime;  //蜂鸣器指示电机状态切换时间

u8 cpflag; //允许取消配对变量

VolStatus BoradVol = VOL_NONE;  //主板电量
BoardStatus BoardSt = NORMAL;   //主板状态
WarningLevel warnlv = NOWARN;   //报警等级

//u32 BigCurrenttime; //大电流报警时间
__IO u16 TempTime;   //温度报警时间
__IO u16 WpTime; //大功率报警时间

__IO u16 beepwarntime;  //蜂鸣器报警时间
__IO u32 beepwarnontime; //蜂鸣器报警持续时间
__IO u16 ledwarntime;	//LED报警时间
__IO u16 ledwarnmaxtime;  //led报警时间间隔

__IO u8 yelflashtimes;  //黄灯闪烁次数
__IO u8 redflashtimes;  //红灯闪烁次数
__IO u8 yeltemptimes;   //替换变量
__IO u8 redtemptimes;   //替换变量

__IO u32 keypairtime; //配对按键按下时间
__IO u32 pairtime;    //配对持续时间

//u8 ControlDevice;  //当前控制转向电机的设备
u8 OrtateMotorLock; //转向电机锁 主要用于转270度停止
__IO u16 OrtateMotorTime; //一次最多转动270度

u8 Voldisflag; //电量显示变量

CommandBuf rxbuf;   //接收缓存
ConnectDev condev;  //连接设备列表

float NowTemp;   //当前温度
//float NowCur;    //当前电流
float NowVol;    //当前电压
float NowWp;     //当前功率

__IO u16 StartTime;
__IO u32 SystemTime; //系统运行时间计数

/********************************
时间计数函数
功能：
	对相关时间进行计数     
	单位：1ms
	
参数：
	无
	
返回值：
	无
**************************************/
void WarningTimeCounter(void)
{
	if(pwr_status == BOOT_RUN)
	{
		if(beepwarntime > 0)
			beepwarntime++;
		if(ledwarntime > 0)
			ledwarntime++;
		if(TempTime > 0)
			TempTime++;
		if(BeepIndTime > 0)
			BeepIndTime++;
		if(OrtateMotorTime > 0)
			OrtateMotorTime++;
		if(beepwarnontime > 0)
			beepwarnontime--;
		if(WpTime > 0)
			WpTime++;
		SystemTime++;
	}
	else if(pwr_status == BOOT_INIT)
	{
		if(StartTime > 0)
			StartTime++;
	}
}

/********************************
警告处理函数
功能：
	对相关的状态进行指示灯及蜂鸣器的设置
	
参数：
	无
	
返回值：
	无
**************************************/

void WarningHandler(void)
{
	if(BoardSt == ST_PAIR || BoardSt == ST_CANCELPAIR)
	{
		if(ledwarntime > 0 && ledwarntime < ledwarnmaxtime)
		{
			RGBGREEN = 1;
			RGBRED = 0;
			RGBBLUE = 0;
		}
		else if(ledwarntime >= ledwarnmaxtime && ledwarntime < (ledwarnmaxtime * 2))
		{
			RGBBLUE = 0;
			RGBGREEN = 0;
			RGBRED = 0;
		}
		else
		{
			ledwarntime = 1;
		}
	}
	
	if(BoardSt >= VOL_FAULT && BoardSt <= TEMP_FAULT)
	{
		if(beepwarnontime > 0)
		{
			if(beepwarntime > 0 && beepwarntime < BEEP_ONTIME)
			{
				BEEP = 1;
			}
			else if(beepwarntime >= BEEP_ONTIME && beepwarntime < (BEEP_ONTIME + BEEP_OFFTIME))
			{
				BEEP = 0;
			}
			else if(beepwarntime >= (BEEP_ONTIME + BEEP_OFFTIME) && beepwarntime < (BEEP_ONTIME * 2 + BEEP_OFFTIME))
			{
				BEEP = 1;
			}
			else if(beepwarntime >= (BEEP_ONTIME * 2 + BEEP_OFFTIME) && beepwarntime < (BEEP_ONTIME * 2 + BEEP_OFFTIME + BEEP_DELAY))
			{
				BEEP = 0;
			}
			else
			{
				beepwarntime = 1;
			}
		}
		else
		{
			BEEP = 0;
		}

		if(yelflashtimes > 0)
		{
			if(ledwarntime > 0 && ledwarntime < ledwarnmaxtime)
			{
				RGBGREEN = 1;
				RGBRED = 1;
				RGBBLUE = 0;
			}
			else if(ledwarntime >= ledwarnmaxtime && ledwarntime < (ledwarnmaxtime * 2))
			{
				RGBBLUE = 0;
				RGBGREEN = 0;
				RGBRED = 0;
			}
			else
			{
				ledwarntime = 1;
				yelflashtimes--;
			}
		}
		else if(redflashtimes > 0)
		{
			if(ledwarntime > 0 && ledwarntime < ledwarnmaxtime)
			{
				RGBGREEN = 0;
				RGBRED = 1;
				RGBBLUE = 0;
			}
			else if(ledwarntime >= ledwarnmaxtime && ledwarntime < (ledwarnmaxtime * 2))
			{
				RGBBLUE = 0;
				RGBGREEN = 0;
				RGBRED = 0;
			}
			else
			{
				ledwarntime = 1;
				redflashtimes--;
			}
		}
		else
		{
			yelflashtimes = yeltemptimes;
			redflashtimes = redtemptimes;
		}
	}

	if(BoardSt == ORTATE_FAULT || BoardSt == WORKPOWER_FAULT)
	{
		if(beepwarnontime > 0)
		{
			if(beepwarntime > 0 && beepwarntime < BEEP_ONTIME)
			{
				BEEP = 1;
			}
			else if(beepwarntime >= BEEP_ONTIME && beepwarntime < (BEEP_ONTIME + BEEP_OFFTIME))
			{
				BEEP = 0;
			}
			else if(beepwarntime >= (BEEP_ONTIME + BEEP_OFFTIME) && beepwarntime < (BEEP_ONTIME * 2 + BEEP_OFFTIME))
			{
				BEEP = 1;
			}
			else if(beepwarntime >= (BEEP_ONTIME * 2 + BEEP_OFFTIME) && beepwarntime < (BEEP_ONTIME * 2 + BEEP_OFFTIME + BEEP_DELAY))
			{
				BEEP = 0;
			}
			else
			{
				beepwarntime = 1;
			}
		

			if(yelflashtimes > 0)
			{
				if(ledwarntime > 0 && ledwarntime < ledwarnmaxtime)
				{
					RGBGREEN = 1;
					RGBRED = 1;
					RGBBLUE = 0;
				}
				else if(ledwarntime >= ledwarnmaxtime && ledwarntime < (ledwarnmaxtime * 2))
				{
					RGBBLUE = 0;
					RGBGREEN = 0;
					RGBRED = 0;
				}
				else
				{
					ledwarntime = 1;
					yelflashtimes--;
				}
			}
			else if(redflashtimes > 0)
			{
				if(ledwarntime > 0 && ledwarntime < ledwarnmaxtime)
				{
					RGBGREEN = 0;
					RGBRED = 1;
					RGBBLUE = 0;
				}
				else if(ledwarntime >= ledwarnmaxtime && ledwarntime < (ledwarnmaxtime * 2))
				{
					RGBBLUE = 0;
					RGBGREEN = 0;
					RGBRED = 0;
				}
				else
				{
					ledwarntime = 1;
					redflashtimes--;
				}
			}
			else
			{
				yelflashtimes = yeltemptimes;
				redflashtimes = redtemptimes;
			}
		}
		else
		{
			BEEP = 0;
		}
	}

	if(BoardSt == CONNECT_FAULT || BoardSt == MOTOR_FAULT || BoardSt == REEDKEY_FAULT)
	{
		if(beepwarntime > 0 && beepwarntime < BEEP_ONTIME)
		{
			BEEP = 1;
		}
		else if(beepwarntime >= BEEP_ONTIME && beepwarntime < (BEEP_ONTIME + BEEP_OFFTIME))
		{
			BEEP = 0;
		}
		else if(beepwarntime >= (BEEP_ONTIME + BEEP_OFFTIME) && beepwarntime < (BEEP_ONTIME * 2 + BEEP_OFFTIME))
		{
			BEEP = 1;
		}
		else if(beepwarntime >= (BEEP_ONTIME * 2 + BEEP_OFFTIME) && beepwarntime < (BEEP_ONTIME * 2 + BEEP_OFFTIME * 2))
		{
			BEEP = 0;
		} 
		else if(beepwarntime >= (BEEP_ONTIME * 2 + BEEP_OFFTIME * 2) && beepwarntime < (BEEP_ONTIME * 3 + BEEP_OFFTIME * 2))
		{
			BEEP = 1;
		}
		else if(beepwarntime >= (BEEP_ONTIME * 3 + BEEP_OFFTIME * 2) && beepwarntime < (BEEP_ONTIME * 3 + BEEP_OFFTIME * 2 + BEEP_DELAY))
		{
			BEEP = 0;
		}
		else
		{
			beepwarntime = 1;
		}

		RGBRED = 1;
		RGBBLUE = 0;
		RGBGREEN = 0;
	}
}

/********************************
错误处理函数
功能：
	处理错误及主板相关的状态
	
参数：
	无
	
返回值：
	无
**************************************/
void FaultHandler(void)
{
	switch(BoardSt)
	{
#if 0
		case CURRENT_FAULT60:
			if(warnlv < CURRENT60WARN || warnlv == CURRENT70WARN)
			{
				beepwarntime = 1;
				ledwarntime = 1;
			    ledwarnmaxtime = RGB_SLOW_FLASH;
				warnlv = CURRENT60WARN;
				if(BigCurrenttime == 0)
				{
					
BigCurrenttime = 1;
				}
			}
			break;
		case CURRENT_FAULT70:
			if(warnlv < CURRENT70WARN)
			{
				beepwarntime = 1;
				ledwarntime = 1;
			    ledwarnmaxtime = RGB_QUIK_FLASH;
				warnlv = CURRENT70WARN;
				if(BigCurrenttime == 0)
				{
					
BigCurrenttime = 1;
				}
			}
			break;
#endif
		case ST_PAIR:
			if(warnlv < PAIRWARN)
			{
				ledwarntime = 1;
				ledwarnmaxtime = RGB_QUIK_FLASH;
				warnlv = PAIRWARN;
			}
			break;
		case ST_CANCELPAIR:
			if(warnlv < CANCELPAIRWARN)
			{
				ledwarntime = 1;
				ledwarnmaxtime = RGB_NORMAL_FLASH;
				warnlv = CANCELPAIRWARN;
			}
			break;
		case WORKPOWER_FAULT:
			if(warnlv < WORKPOWERWARN)
			{
				beepwarntime = 1;
				ledwarntime = 1;
			    ledwarnmaxtime = RGB_QUIK_FLASH;
				yelflashtimes = 1;
				redflashtimes = RGB_FLASH_TIMES - yelflashtimes;
				yeltemptimes = yelflashtimes;
				redtemptimes = redflashtimes;
				if(Speed != SPEED0)
				{
					beepwarnontime = 60000;
				}
				MotorMoveSpeedSet();
				warnlv = WORKPOWERWARN;
			}
			break;
		case VOL_FAULT:
		case HIGH_VOL_FAULT:
			if(warnlv < VOLWARN)
			{
				beepwarntime = 1;
				ledwarntime = 1;
			    ledwarnmaxtime = RGB_QUIK_FLASH;
				yelflashtimes = 3;
				redflashtimes = RGB_FLASH_TIMES - yelflashtimes;
				yeltemptimes = yelflashtimes;
				redtemptimes = redflashtimes;
				beepwarnontime = 60000;
				warnlv = VOLWARN;
			}
			break;
		case ORTATE_FAULT:
			if(warnlv < ORTATEWARN)
			{
				beepwarntime = 1;
				ledwarntime = 1;
			    ledwarnmaxtime = RGB_QUIK_FLASH;
				yelflashtimes = 4;
				redflashtimes = RGB_FLASH_TIMES - yelflashtimes;
				yeltemptimes = yelflashtimes;
				redtemptimes = redflashtimes;
				beepwarnontime = 10000;
				Ortate_Motor_Brate();
				MotorMoveStop();
				warnlv = ORTATEWARN;
			}
			break;
		case TEMP_FAULT:
			if(warnlv < TEMPWARN)
			{
				beepwarntime = 1;
				ledwarntime = 1;
			    ledwarnmaxtime = RGB_QUIK_FLASH;
				yelflashtimes = 2;
				redflashtimes = RGB_FLASH_TIMES - yelflashtimes;
				yeltemptimes = yelflashtimes;
				redtemptimes = redflashtimes;
				beepwarnontime = 60000;
				warnlv = TEMPWARN;
			}
			break;
		case REEDKEY_FAULT:
		case MOTOR_FAULT:
			if(warnlv < MOTORWARN)
			{
				Ortate_Motor_Brate();
				MotorMoveStop();
				DeviceMode = INCH_MODE;
				warnlv = MOTORWARN;
			}
			break;
		case CONNECT_FAULT:
			if(warnlv < CONNECTWARN)

			{
				Ortate_Motor_Brate();
				MotorMoveStop();
				DeviceMode = INCH_MODE;
				warnlv = CONNECTWARN;
			}
			break;
		case NORMAL:
			if(condev.connum > 0)
			{
				if(MoveMotorStatus != MOTORMOVESTOP)
				{
					RGBBLUE = 1;
					RGBGREEN = 0;
				}
				else
				{
					RGBBLUE = 0;
					RGBGREEN = 1;
				}
				RGBRED = 0;
			}
			else
			{
				RGBBLUE = 1;
				RGBGREEN = 1;
				RGBRED = 1;
			}
			if(warnlv != NOWARN)
				BEEP = 0;
			beepwarntime = 0;
			ledwarntime = 0;
//			BigCurrenttime = 0;
			yelflashtimes = 0;
			redflashtimes = 0;
//			yeltemptimes = yelflashtimes;
//			redtemptimes = redflashtimes;
			beepwarnontime = 0;
			TempTime = 0;
//			WpTime = 0;
			warnlv = NOWARN;
		default:
			break;
	}
	WarningHandler();
}

/********************************
电压处理函数
功能：
	处理电压
	
参数：
	无
	
返回值：
	无
**************************************/
void VoltageHandler(float vol)
{
	VolStatus st;
	if(fabs(vol - NowVol) >= VOLCHANGEVAL)
	{
		NowVol = vol;
		if(vol > VOLTAGE4)
		{
			if(vol >= VOLTAGEMAX)
			{
				if(BoardSt < HIGH_VOL_FAULT)
				{
					BoardSt = HIGH_VOL_FAULT;
				}
			}
			else
			{
				if(BoardSt <= HIGH_VOL_FAULT)
				{
					BoardSt = VOL_FAULT;
				}
			}
		}
		else if(vol <= VOLTAGE4 && vol > VOLTAGE3)
		{
			st = VOL_FULL;
			if(BoardSt == VOL_FAULT || BoardSt == HIGH_VOL_FAULT)
			{
				BoardSt = NORMAL;
			}
			if(st != BoradVol)
			{
				if(Voldisflag == 0)
				{
					LED1 = 1;
					LED2 = 1;
					LED3 = 1;
					LED4 = 1;
				}
				BoradVol = st;
			}
		}
		else if(vol <= VOLTAGE3 && vol > VOLTAGE2)
		{
			st = VOL_LEVEL3;
			if(BoardSt == VOL_FAULT || BoardSt == HIGH_VOL_FAULT)
			{
				BoardSt = NORMAL;
			}
			if(st != BoradVol)
			{
				
				if(Voldisflag == 0)
				{
					LED1 = 1;
					LED2 = 1;
					LED3 = 1;
					LED4 = 0;
				}
				BoradVol = st;
			}
		
		}
		else if(vol <= VOLTAGE2 && vol > VOLTAGE1)
		{
			st = VOL_LEVEL2;
			if(BoardSt == VOL_FAULT || BoardSt == HIGH_VOL_FAULT)
			{
				BoardSt = NORMAL;
			}
			if(st != BoradVol)
			{
				if(Voldisflag == 0)
				{
					LED1 = 1;
					LED2 = 1;
					LED3 = 0;
					LED4 = 0;
				}
				BoradVol = st;
			}
		
		}
		else if(vol <= VOLTAGE1 && vol > VOLTAGE0)
		{
			st = VOL_LEVEL1;
			if(BoardSt == VOL_FAULT || BoardSt == HIGH_VOL_FAULT)
			{
				BoardSt = NORMAL;
			}
			if(st != BoradVol)
			{
				if(Voldisflag == 0)
				{
					LED1 = 1;
					LED2 = 0;
					LED3 = 0;
					LED4 = 0;
				}
				BoradVol = st;
			}
		
		}
		else
		{
			if(Voldisflag == 0)
			{
				LED1 = 0;
				LED2 = 0;
				LED3 = 0;
				LED4 = 0;
			}
			if(BoardSt < VOL_FAULT)
			{
				BoardSt = VOL_FAULT;
			}
			BoradVol = VOL_NONE;
		}
	}
}

/********************************
功率处理
功能：
	功率处理
	
参数：
	无
	
返回值：
	无
**************************************/
void WorkPowerHandler(float cur,float vol)
{
	float wp;
	wp = cur * vol;
	
	if(MoveMotorStatus == MOTORMOVESTOP && OrtateMotorStatus == ORTATE_STATUS_STOP)
	{
		if(fabs(wp - NowWp) >= WPCHANGEVAL)
		{
			if(wp >= 50)    //电机停止状态下大于50W报警
			{
				BoardSt = MOTOR_FAULT;
				return;
			}
		}
	}

	if(BoardSt == WORKPOWER_FAULT && cur > 30 && (now_speed == target_speed))
	{
		MotorMoveStop();
	}
	
	if(fabs(wp - NowWp) >= WPCHANGEVAL)
	{
		NowWp = wp;
		if(wp > 600 && wp < 720)    //功率大于600W报警
		{
			if(WpTime == 0 && BoardSt < WORKPOWER_FAULT)
			{
				WpTime = 1;
			}
//			printf("%d\r\n",WpTime);
			if(WpTime > 30000)
			{
				WpTime = 0;
				if(BoardSt <= WORKPOWER_FAULT)
				{
					BoardSt = WORKPOWER_FAULT;
					if(Speed != SPEED0)
					{
						beepwarnontime = 60000;
					}

				}
			}
		}
		else if(wp >= 720)
		{
			if(BoardSt <= WORKPOWER_FAULT)
			{
				BoardSt = WORKPOWER_FAULT;
				WpTime = 0;
				if(Speed != SPEED0)
				{
					beepwarnontime = 60000;
				}
			
			}
		}
		else
		{
			if(BoardSt == WORKPOWER_FAULT && beepwarnontime == 0)
			{
				BoardSt = NORMAL;
//				MotorMoveSpeedSet();
			}
		}
	}
}

#if 0
void CurrentHandler(float cur)
{
	if(fabs(cur - NowCur) >= CURCHANGEVAL)
	{
		NowCur = cur;
		if(cur > 60)
		{
			if(cur > 70)
			{
				if(BoardSt < CURRENT_FAULT70)
				{
					BoardSt = CURRENT_FAULT70;
				}
			}
			else
			{
				if(BoardSt <= CURRENT_FAULT70)
				{
					BoardSt = CURRENT_FAULT60;
				}
			}
			
		}
		else
		{
			if(BoardSt == CURRENT_FAULT60 || BoardSt == CURRENT_FAULT70)
			{
				BoardSt = NORMAL;
			}
		}
	}
}
#endif

/********************************
温度处理
功能：
	温度处理
	
参数：
	无
	
返回值：
	无
**************************************/
void TempHandler(float temp)
{
	if(fabs(temp - NowTemp) >= TEMPCHANGEVAL)
	{
		NowTemp = temp;
/*
		if(temp <= TEMP80)
		{
			if(TempTime == 0)
			{
				TempTime = 1;
			}

			if(TempTime > 30000)
			{
				if(BoardSt < TEMP_FAULT)
				{
					BoardSt = TEMP_FAULT;
				}
			}

		}
		else
		{
			if(BoardSt == TEMP_FAULT)
			{
				BoardSt = NORMAL;
			}
		}
*/
	}
}

void TempCheck(void)
{
	if(NowTemp <= TEMP80)
	{
		if(TempTime == 0)
		{
			TempTime = 1;
		}
	
		if(TempTime > 30000)
		{
			if(BoardSt < TEMP_FAULT)
			{
				BoardSt = TEMP_FAULT;
			}
		}
	
	}
	else
	{
		if(BoardSt == TEMP_FAULT)
		{
			BoardSt = NORMAL;
		}
	}
}

void VolCheck(void)
{
	if(NowVol > VOLTAGE4)
	{
		if(NowVol >= VOLTAGEMAX)
		{
			if(BoardSt < HIGH_VOL_FAULT)
			{
				BoardSt = HIGH_VOL_FAULT;
			}
		}
		else
		{
			if(BoardSt <= HIGH_VOL_FAULT)
			{
				BoardSt = VOL_FAULT;
			}
		}
	}
}

/********************************
转向电机错误处理
功能：
	转向电机错误处理
	
参数：
	无
	
返回值：
	无
**************************************/
void OrtateFaultCheck(void)
{
	if(nF == 0)
	{
		if(BoardSt <= ORTATE_FAULT)
		{
			BoardSt = ORTATE_FAULT;
			beepwarnontime = 10000;
		}
	}
	else
	{
		if(BoardSt == ORTATE_FAULT && beepwarnontime == 0)
		{
			
			BoardSt = NORMAL;
		}
	}
}

/********************************
跌倒开关错误确认函数
功能：
	跌倒开关错误处理
	
参数：
	无
	
返回值：
	无
**************************************/
void ReedkeyFaultCheck(void)
{
	if(REED_KEY == 1)
	{
		if(BoardSt < REEDKEY_FAULT)
			BoardSt = REEDKEY_FAULT;
	}
	else
	{
		if(BoardSt == REEDKEY_FAULT)
		{
			BoardSt = NORMAL;
		}
	}

}


/********************************
ADC采样处理函数
功能：
	处理ADC采样出来的数据，温度，功率，电压
	
参数：
	无
	
返回值：
	无
**************************************/
void ADCHandler(void)
{
	u16 *val;
	float vol;
	float cur;
	float temp;
	if(SystemTime % 1000 == 0)  //1S处理一次
	{
//		DEBUGMSG("PB3 = %d pb4 = %d",KEY_PAIR,KEY_PWR);
		DEBUGMSG("BOARD_ST %d", BoardSt);

		val = Get_ADC_Value();
//		printf("val = %d %d %d\r\n",val[0],val[1],val[2]);
		cur = val[0] * 3.3 / 4096 / 20 * 1000 / 2; //工作电流
		vol = val[1] * 3.3 / 4096 * 10;  //工作电压
		temp = val[2] * 3.3 / 4096 / (3.3 - (val[2] * 3.3 / 4096)) * 5.1; //这里算出来的是热敏电阻阻值 单位：千欧
		
		cur = (cur - CUR_NOISE) * CUR_BIAS;
		if(cur < 0)
		{
			cur = 0;
		}
//		printf("vol = %f cur = %f temp = %f\r\n",vol,cur,temp);
		VoltageHandler(vol);
	//	CurrentHandler(cur);
		WorkPowerHandler(cur,vol);

		TempHandler(temp);
	}
	TempCheck();
	VolCheck();
	OrtateFaultCheck();
	ReedkeyFaultCheck();
}

/********************************
电机移动时间计数函数
功能：
	时间计数
	
参数：
	无
	
返回值：
	无
**************************************/
void MotorMoveCounter(void)
{
	if(pwr_status == BOOT_RUN)
	{
		if(MotorMoveTime > 0)
		{
			MotorMoveTime++;
		}

		if(OrateMoveTime > 0)
		{
			OrateMoveTime++;
		}
	}
}

/********************************
相关参数初始化
功能：
	相关参数初始化
	
参数：
	无
	
返回值：
	无
**************************************/
void DeviceStatusInit(void)
{
	Voldisflag = 0;
	cpflag = 0;
//	ControlDevice = MAIN_BOARD;
	keypairtime = 0;
	pairtime = 0;
	SystemTime = 0;
	StartTime = 0;
	BeepIndTime = 0;
	OrtateMotorLock = 0;
	OrtateMotorTime = 0;
	Speed = SPEED0;
	now_speed = 0;
	target_speed = 0;
	Speed_temp = Speed;
	NowTemp = 0;
//	NowCur = 0;
	NowVol = 0;
	NowWp = 0;
	MotorMoveTime = 0;
	OrateMoveTime = 0;
//	BigCurrenttime = 0;
//	OrateMotorStatus = MOTORMOVESTOP;
	pwr_status = BOOT_STOP;  //停机开机管理变量
	pwr_time = 0; //关机时间计数
	ConnStatus = DEVINIT;
	DeviceMode = INCH_MODE;
	MoveMotorStatus = MOTORMOVESTOP;
	OrtateMotorStatus = ORTATE_STATUS_STOP;
	BoardSt = NORMAL;
	warnlv = NOWARN;
	memset(&rxbuf,0,sizeof(rxbuf));
	memset(&condev,0,sizeof(condev));
}

/********************************
前进电机启动
功能：
	前进电机启动并设置相关速度
	
参数：
	无
	
返回值：
	无
**************************************/
void MotorMoveStart(void)
{
	u8 st;
	st = (DeviceMode << 1) + 1;
	TIM2->CCER |= 1 << 4;
	MotorMoveSpeedSet();
	MoveMotorStatus = MOTORMOVERUN;
	CAN_Send_Msg(&st, 1, MAIN_BOARD, MOTOR_CHANGE);
}

/********************************
前进电机停止
功能：
	前进电机停止
	
参数：
	无
	
返回值：
	无
**************************************/
void MotorMoveStop(void)
{
	u8 st;
	st = (DeviceMode << 1);
	now_speed = 0;
	target_speed = 0;
	Speed = 0;
	TIM_SetCompare2(TIM2, now_speed);
	TIM2->CCER &=  0xffef;
	MoveMotorStatus = MOTORMOVESTOP;
	CAN_Send_Msg(&st, 1, MAIN_BOARD, MOTOR_CHANGE);
}

/********************************
前进电机速度设置
功能：
	前进电机速度设置
	
参数：
	无
	
返回值：
	无
**************************************/
#if 1
void MotorMoveSpeedSet(void)
{
	switch(Speed)
	{
		case SPEED0:
			target_speed = SPEED0_VAL;
			break;
		case SPEED1:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				target_speed = SPEED1_VAL/2;
			}
			else
			{
				target_speed = SPEED1_VAL;
			}
			break;
		case SPEED2:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				target_speed = SPEED2_VAL/2;
			}
			else
			{
				target_speed = SPEED2_VAL;
			}
			break;
		case SPEED3:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				target_speed = SPEED3_VAL/2;
			}
			else
			{
				target_speed = SPEED3_VAL;
			}
			break;
		case SPEED4:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				target_speed = SPEED4_VAL/2;
			}
			else
			{
				target_speed = SPEED4_VAL;
			}
			break;
		case SPEED5:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				target_speed = SPEED5_VAL/2;
			}
			else
			{
				target_speed = SPEED5_VAL;
			}
			break;
		case SPEED6:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				target_speed = SPEED6_VAL/2;
			}
			else
			{
				target_speed = SPEED6_VAL;
			}
			break;
		case SPEED7:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				target_speed = SPEED7_VAL/2;
			}
			else
			{
				target_speed = SPEED7_VAL;
			}
			break;
		default:
			break;
	}
}
#else
void MotorMoveSpeedSet(void)
{
	u8 spd;
	spd = Speed;
	
	switch(spd)
	{
		case SPEED0:
			if(BoardSt == WORKPOWER_FAULT)
			{
				beepwarnontime = 0;
			}
			TIM_SetCompare2(TIM2, SPEED0_VAL);
			break;
		case SPEED1:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				TIM_SetCompare2(TIM2, SPEED1_VAL/2);
			}
			else
			{
				TIM_SetCompare2(TIM2, SPEED1_VAL);
			}
			break;
		case SPEED2:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				TIM_SetCompare2(TIM2, SPEED2_VAL/2);
			}
			else
			{
				TIM_SetCompare2(TIM2, SPEED2_VAL);
			}
			break;
		case SPEED3:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				TIM_SetCompare2(TIM2, SPEED3_VAL/2);
			}
			else
			{
				TIM_SetCompare2(TIM2, SPEED3_VAL);
			}
			break;
		case SPEED4:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				TIM_SetCompare2(TIM2, SPEED4_VAL/2);
			}
			else
			{
				TIM_SetCompare2(TIM2, SPEED4_VAL);
			}
			break;
		case SPEED5:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				TIM_SetCompare2(TIM2, SPEED5_VAL/2);
			}
			else
			{
				TIM_SetCompare2(TIM2, SPEED5_VAL);
			}
			break;
		case SPEED6:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				TIM_SetCompare2(TIM2, SPEED6_VAL/2);
			}
			else
			{
				TIM_SetCompare2(TIM2, SPEED6_VAL);
			}
			break;
		case SPEED7:
			if(BoardSt == WORKPOWER_FAULT || BoardSt == TEMP_FAULT)
			{
				TIM_SetCompare2(TIM2, SPEED7_VAL/2);
			}
			else
			{
				TIM_SetCompare2(TIM2, SPEED7_VAL);
			}
			break;
		default:
			break;
	}
}
#endif

void SpeedChange(void)
{
	if(MoveMotorStatus == MOTORMOVERUN)
	{
		if(now_speed < target_speed)
		{
//			printf("now_speed = %d target_speed = %d\r\n",now_speed,target_speed);
			now_speed += SPEED_SWITCH_BASE;
			TIM_SetCompare2(TIM2, now_speed);
		}
		else if(now_speed > target_speed)
		{
			now_speed = target_speed;
			TIM_SetCompare2(TIM2, target_speed);
		}
	}
}
/********************************
心跳计数
功能：
	心跳计数
	
参数：
	无
	
返回值：
	无
**************************************/
void HeartBeatCounter(void)
{
	if(pwr_status == BOOT_RUN)
	{
		for(u8 i = 0;i < MAX_CON_DEVNUM;i++)
		{
			if(condev.dev[i].status == DEVCONN)
			{
				condev.dev[i].heart_time++;
			}
		}	
	}
}

/********************************
连接设备搜索
功能：
	搜索设备是否已连接
	
参数：
	设备ID
	
返回值：
	成功：储存位置
	失败：0xff
**************************************/
u8 SearchDevice(u8 devid)
{
	if(condev.dev[devid].status == DEVCONN)
	{
		return devid;
	}
	DEBUGMSG("SearchDevice fail");
	return 0xff;
}

void ConnectCheck(void)
{
	for(u8 i = 0;i < MAX_CON_DEVNUM;i++)
	{
		if(condev.dev[i].heart_time > MAX_HEART_TIME)
		{
			if(condev.dev[i].status == DEVCONN)
			{
				condev.dev[i].status = DEVLOSE;
				BoardSt = CONNECT_FAULT;
				condev.connum--;
			}
			condev.dev[i].heart_time = 1;
		}
	}
}

/********************************
刷新心跳
功能：
	刷新心跳
	
参数：
	设备ID
	
返回值：
	无
**************************************/
void ReflashHeartBeat(u8 devid)
{
	u8 i;
	if((i = SearchDevice(devid)) != 0xff)
	{
		condev.dev[i].heart_time = 1;
	}
	else
	{
		if(condev.dev[i].status == DEVLOSE)
		{
			condev.dev[i].status = DEVCONN;
			condev.dev[i].heart_time = 1;
		}
	}
}

/********************************
心跳处理函数
功能：
	心跳处理
	
参数：
	设备ID
	
返回值：
	无
**************************************/
void HeartBeatHandler(u8 devid)
{
	ReflashHeartBeat(devid);
}

/********************************
连接处理函数
功能：
	连接处理
	
参数：
	相关命令结构体
	
返回值：
	无
**************************************/
void ConnectHandler(CommandData* dev)
{
//	if(SearchDevice(dev->dev_id) == 0xff)
//	{
		if(BoardSt == CONNECT_FAULT)
		{
			if(condev.dev[dev->dev_id].status == DEVLOSE)
			{
				BoardSt = NORMAL;
			}
		}
		
		if(condev.dev[dev->dev_id].status != DEVCONN)
		{
			condev.dev[dev->dev_id].dev_id = dev->dev_id;
			condev.dev[dev->dev_id].status = DEVCONN;
			condev.connum++;
		}
		
		condev.dev[dev->dev_id].heart_time = 1;
		CAN_Send_Msg(NULL, 0, condev.dev[dev->dev_id].dev_id, CONNECT_ACK);
//	}
}

/********************************
模式改变
功能：
	改变控制模式
	
参数：
	相关命令结构体
	
返回值：
	无
**************************************/
void ModeChangeHandler(CommandData* dev)
{
	if(SearchDevice(dev->dev_id) != 0xff)
	{
		if(BoardSt <= WORKPOWER_FAULT && BoardSt != HIGH_VOL_FAULT && BoardSt != ORTATE_FAULT && BoardSt != ST_PAIR && BoardSt != ST_CANCELPAIR)
		{
			if(DeviceMode == INCH_MODE)
			{
				DeviceMode = LINK_MODE;
				MotorMoveStart();
			}
			else
			{
				DeviceMode = INCH_MODE;
				MotorMoveStop();
			}
			
			if(BoardSt == NORMAL)

			{
				BeepIndTime = 1;
				BEEP = 1;
			}

			MotorMoveTime = 0;
		}
		ReflashHeartBeat(dev->dev_id);
	}
}

/********************************
转向电机控制
功能：
	控制转向
	
参数：
	相关命令结构体
	
返回值：
	无
**************************************/
void OrtateMotorControl(CommandData* dev)
{
	if(SearchDevice(dev->dev_id) != 0xff)
	{
		if(BoardSt < CONNECT_FAULT && BoardSt != ORTATE_FAULT && BoardSt != ST_PAIR && BoardSt != ST_CANCELPAIR)
		{
			DEBUGMSG("OrtateMotorControl");
			
			if(dev->dev_cmd == LEFT_TURN)
			{
//				if(ControlDevice == MAIN_BOARD || ControlDevice == dev->dev_id)
//				{
					if(OrtateMotorLock == 0)
					{
						if(OrtateMotorStatus == ORTATE_STATUS_RIGHT)
						{
							Ortate_Motor_Brate();
							OrtateMotorLock = 1;
							OrtateMotorTime = 0;
							OrateMoveTime = 0;
						}
						else if(OrtateMotorStatus == ORTATE_STATUS_LEFT)
						{
							OrateMoveTime = 1;
						}
						else if(OrtateMotorStatus == ORTATE_STATUS_STOP)
						{
							if(BoardSt == NORMAL)

							{
								BeepIndTime = 1;
								BEEP = 1;
							}
							Ortate_Motor_Left();
							OrateMoveTime = 1;
							OrtateMotorTime = 1;
						}

					}
//					ControlDevice = dev->dev_id;
//				}
			}
			else if(dev->dev_cmd == RIGHT_TURN)
			{
//				if(ControlDevice == MAIN_BOARD || ControlDevice == dev->dev_id)
//				{
					if(OrtateMotorLock == 0)
					{
						if(OrtateMotorStatus == ORTATE_STATUS_LEFT)
						{
							Ortate_Motor_Brate();
							OrtateMotorLock = 1;
							OrtateMotorTime = 0;
							OrateMoveTime = 0;
						}
						else if(OrtateMotorStatus == ORTATE_STATUS_RIGHT)
						{
							OrateMoveTime = 1;
						}
						else if(OrtateMotorStatus == ORTATE_STATUS_STOP)
						{
							if(BoardSt == NORMAL)

							{
								BeepIndTime = 1;
								BEEP = 1;
							}

							Ortate_Motor_Right();
							OrateMoveTime = 1;
							OrtateMotorTime = 1;
						}

					}
//					ControlDevice = dev->dev_id;
//				}
			}
			else
			{	
/*				if(BoardSt == NORMAL)

				{
					BeepIndTime = 1;
					BEEP = 1;
				}  */
//				if(OrtateMotorStatus != ORTATE_STATUS_STOP)
//				{
					Ortate_Motor_Brate();
					OrtateMotorLock = 0;
					OrtateMotorTime = 0;
					OrateMoveTime = 0;
//					ControlDevice = MAIN_BOARD;
//				}
			}
		}
		ReflashHeartBeat(dev->dev_id);
	}
}

/********************************
电机移动控制
功能：
	电机移动控制
	
参数：
	相关命令结构体
	
返回值：
	无
**************************************/
void MotorMoveControlHandler(CommandData* dev)
{
	if(SearchDevice(dev->dev_id) != 0xff)
	{
		if(BoardSt <= WORKPOWER_FAULT && BoardSt != HIGH_VOL_FAULT && BoardSt != ORTATE_FAULT && BoardSt != ST_PAIR && BoardSt != ST_CANCELPAIR)
		{
			if(DeviceMode == INCH_MODE)
			{
				if(dev->dev_cmd == START_MOVE)
				{
					if(MoveMotorStatus == MOTORMOVERUN)
					{
						MotorMoveTime = 1;
					}
					else
					{
						if(BoardSt == NORMAL)

						{
							BeepIndTime = 1;
							BEEP = 1;
						}

//						Speed = dev->data[0];
//						MotorMoveSpeedSet();
						MotorMoveStart();
						MotorMoveTime = 1;
					}
				}
				else if(dev->dev_cmd == STOP_MOVE)
				{
	/*				if(BoardSt == NORMAL)

					{
						BeepIndTime = 1;
						BEEP = 1;
					}*/

					MotorMoveTime = 0;
					MotorMoveStop();	
				}
				else
				{
					return;		
				}
			}
		}
		ReflashHeartBeat(dev->dev_id);
	}
}

/********************************
电机速度控制
功能：
	电机速度控制
	
参数：
	相关命令结构体
	
返回值：
	无
**************************************/
void SpeedControlHandler(CommandData* dev)
{
	if(SearchDevice(dev->dev_id) != 0xff)
	{
		if(BoardSt != ST_PAIR && BoardSt != ST_CANCELPAIR)
		{
			if(Speed != dev->data[0])
			{
				if(dev->data[0] <= SPEED7)
				{
					Speed = dev->data[0];
				}
				else
				{
					if(DeviceMode == LINK_MODE)
					{
						if(dev->data[0] == SPEED_ADD)
						{
							if(Speed < SPEED7)
							{
								Speed++;
							}
							else
							{
								ReflashHeartBeat(dev->dev_id);
								return;
							}
						}
						else if(dev->data[0] == SPEED_SUB)
						{
							if(Speed > SPEED0)
							{
								Speed--;
							}
							else
							{
								ReflashHeartBeat(dev->dev_id);
								return;
							}
						}
						else if(dev->data[0] == SPEED_ONMAX)
						{
							if(Speed < SPEED7)
							{
								Speed_temp = Speed;
								Speed = SPEED7;
							}
							else
							{
								Speed = Speed_temp;
							}
						}
					}
				}
				
				if(BoardSt == NORMAL)
				{
					BeepIndTime = 1;
					BEEP = 1;
				}
				MotorMoveSpeedSet();
			}
		}
		ReflashHeartBeat(dev->dev_id);
	}
}

//退出配对状态
void QuitPair(void)
{
	if(BoardSt == ST_PAIR || BoardSt == ST_CANCELPAIR)
	{
		BoardSt = NORMAL;
		pairtime = 0;
		cpflag = 0;
	}
}

/************************************************
配对应答处理函数
功能：
	配对应答处理
	
参数：
	无

返回值：
	无
*************************************************/
void PairAckHandler(CommandData* dev)
{
	QuitPair();
	ReflashHeartBeat(dev->dev_id);
}

/************************************************
按键状态处理函数
功能：
	根据按键状态处理相关的事件
	
参数：
	无

返回值：
	无
*************************************************/
void KeyHandler(void)
{
	if(pwr_status == BOOT_RUN)
	{
		if(KPStatus == KPPRESS)
		{
			if(condev.dev[REMOTE_BOARD].status == DEVCONN)
			{
				DEBUGMSG("KPStatus == KPPRESS");
				if(keypairtime == 0)
				{
					if(KeyFlag == 0)
					{
						DEBUGMSG("keypairtime = SystemTime;");
						keypairtime = SystemTime;
						KeyFlag = 1;
					}
				}
				else
				{
					DEBUGMSG("KEYPAIRTOUCH");
				
					if(SystemTime - keypairtime > KEYPAIRTOUCH && pairtime == 0)
					{
						DEBUGMSG("keypairtime");
						if(BoardSt == NORMAL && OrtateMotorStatus == ORTATE_STATUS_STOP && MoveMotorStatus == MOTORMOVESTOP)
						{
							CAN_Send_Msg(NULL, 0, MAIN_BOARD, PAIRING);
						
	BoardSt = ST_PAIR;
							pairtime = SystemTime;
						}

					}

					if(SystemTime - keypairtime > KEYPAIRTOUCH && BoardSt == ST_PAIR && cpflag == 1)
					{
						if(OrtateMotorStatus == ORTATE_STATUS_STOP && MoveMotorStatus == MOTORMOVESTOP)
						{
							CAN_Send_Msg(NULL, 0, MAIN_BOARD, CANCEL_PAIR);
							BoardSt = ST_CANCELPAIR;
							pairtime = SystemTime;
						}
					}
				}
			}
		}
		else if(KPStatus == KPRELEASE)
		{
			DEBUGMSG("BoardSt = %d", BoardSt);
			if(BoardSt == ST_PAIR)
			{
				cpflag = 1;
			}
			KeyFlag = 0;
			keypairtime = 0;
			KPStatus = KEYNONE;
		}

	}

	if(KPWRStatus == KPWRPRESS)
	{
		if(KPWRFlag == 0)
		{
			if(pwr_status == BOOT_STOP)
			{
		/*		if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
				{
					PWR_ClearFlag(PWR_FLAG_WU);
				}
				SYSCLKConfig_STOP();	*/
				pwr_status = BOOT_INIT;
				StartTime = 0;
				pwr_time = 1;
			}
			else if(pwr_status == BOOT_RUN)
			{
				pwr_time = 1;
				Voldisflag = 1;
			}
			else
			{
				pwr_time = 1;
	//				StartTime = 0;
			}
			KPWRFlag = 1;
		}
	}
	else if(KPWRStatus == KPWRRELEASE)
	{
		if(pwr_status == BOOT_INIT)
		{
			if(pwr_time > 100 && pwr_time < VOLDISTOUCH)
			{
				Voldisflag = 1;
			}
			else
			{
				pwr_status = BOOT_STOP;
			}
		}
		else if(pwr_status == BOOT_RUN)
		{
			Volreflash();
			Voldisflag = 0;
		}
		pwr_time = 0;
		KPWRStatus = KEYNONE;
		KPWRFlag = 0;
	}
}


/********************************
电压更新
功能：
	电压更新
	
参数：
	无
	
返回值：
	无
**************************************/
void Volreflash(void)
{
	u16 *val;
	float vol;
	val = Get_ADC_Value();
	vol = val[1] * 3.3 / 4096 * 10;  //工作电压
	Voldisplay(vol);
}

/********************************
开机运行函数
功能：
	开机状态下主程序运行
	
参数：
	无
	
返回值：
	无
**************************************/
void BootRunHandler(void)
{
	if(rxbuf.Rxflag == 1)
	{
		DEBUGMSG("Control_Handler dev_cmd = %d", rxbuf.cmd.dev_cmd);
		switch(rxbuf.cmd.dev_cmd)
		{
			case HEARTBEAT:
				HeartBeatHandler(rxbuf.cmd.dev_id);
				break;
			case LEFT_TURN:
			case RIGHT_TURN:
			case ORTATE_STOP:
				OrtateMotorControl(&rxbuf.cmd);
//					QuitPair();
				break;
			case START_MOVE:
			case RETREAT:
			case STOP_MOVE:
				MotorMoveControlHandler(&rxbuf.cmd);
//					QuitPair();
				break;
			case SPEED_CNTR:
				SpeedControlHandler(&rxbuf.cmd);
//					QuitPair();
				break;
			case MODE_CHANGE:
				ModeChangeHandler(&rxbuf.cmd);
//					QuitPair();
				break;
			case PAIR_ACK:
				PairAckHandler(&rxbuf.cmd);
				break;
			case CONNECT:
				ConnectHandler(&rxbuf.cmd);
				break;
			case CONNECT_ACK:
				break;
			default:
				break;
		}

		rxbuf.Rxflag = 0;
	}
//	SpeedChange();
	if(BoardSt == WORKPOWER_FAULT && now_speed == 0)
	{
		beepwarnontime = 0;
	}

	if(MotorMoveTime > MAX_MOVE_TIME)
	{
		MotorMoveStop();
		MotorMoveTime = 0;
	}

	if(BeepIndTime > BEEPINDMAXTIME)
	{
		BeepIndTime = 0;
		BEEP = 0;
	}

	if(OrateMoveTime > MAX_MOVE_TIME)
	{
		Ortate_Motor_Brate();
//			OrtateMotorLock = 0;
		OrateMoveTime = 0;
		OrtateMotorTime = 0;
//		ControlDevice = MAIN_BOARD;
	}

	if((BoardSt == ST_PAIR || BoardSt == ST_CANCELPAIR) && (SystemTime - pairtime > PAIR_MAX_TIME))
	{
		BoardSt = NORMAL;
		pairtime = 0;
		cpflag = 0;
	}

	if(OrtateMotorTime > ORTATEMOTORTIME)
	{
		Ortate_Motor_Brate();
		OrateMoveTime = 0;
		OrtateMotorLock = 1;
		OrtateMotorTime = 0;
	}
	KeyShakeCheck();
	KeyHandler();
	ConnectCheck();
	ADCHandler();
	FaultHandler();
	CloseBootCheck();
}

/********************************
BOOT_INIT状态处理函数
功能：
	BOOT_INIT状态处理
	
参数：
	无
	
返回值：
	无
**************************************/

void BootInitHandler(void)
{
	

	
	if(Voldisflag)
	{
		Volreflash();
		Voldisflag = 0;
		StartTime = 1;
	}

	if(StartTime > VOLDISTIME)
	{
		StartTime = 0;
		pwr_status = BOOT_STOP;
	}

	if(pwr_time >= VOLDISTOUCH && pwr_time <= (STARTTIME + VOLDISTOUCH))
	{
		StartTime = 0;
		LED1 = 1;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
	}
	else if(pwr_time > (STARTTIME + VOLDISTOUCH) && pwr_time <= (STARTTIME * 2 + VOLDISTOUCH))
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 0;
		LED4 = 0;
	}
	else if(pwr_time > (STARTTIME * 2 + VOLDISTOUCH) && pwr_time <= (STARTTIME * 3 + VOLDISTOUCH))
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 1;
		LED4 = 0;
	}
	else if(pwr_time > (STARTTIME * 3 + VOLDISTOUCH) && pwr_time <= (STARTTIME * 4 + VOLDISTOUCH))
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 1;
		LED4 = 1;
	}
	else if(pwr_time > (STARTTIME * 4 + VOLDISTOUCH))
	{
		pwr_time = 0;
		pwr_status = BOOT_RUN;
//			StartTime = 0;
		CAN_Send_Msg(NULL, 0, MAIN_BOARD,START_BOOT);
	}
}

/********************************
关机确认函数
功能：
	播放关机动画
	
参数：
	无
	
返回值：
	无
**************************************/
void CloseBootCheck(void)
{
	if(Voldisflag == 1)
	{
		if(pwr_time >= VOLDISTOUCH && pwr_time <= (STARTTIME + VOLDISTOUCH))
		{
			LED1 = 1;
			LED2 = 1;
			LED3 = 1;
			LED4 = 1;
		}
		else if(pwr_time > (STARTTIME + VOLDISTOUCH) && pwr_time <= (STARTTIME * 2 + VOLDISTOUCH))
		{
			LED1 = 1;
			LED2 = 1;
			LED3 = 1;
			LED4 = 0;
		}
		else if(pwr_time > (STARTTIME * 2 + VOLDISTOUCH) && pwr_time <= (STARTTIME * 3 + VOLDISTOUCH))
		{
			LED1 = 1;
			LED2 = 1;
			LED3 = 0;
			LED4 = 0;
		}
		else if(pwr_time > (STARTTIME * 3 + VOLDISTOUCH) && pwr_time <= (STARTTIME * 4 + VOLDISTOUCH))
		{
			LED1 = 1;
			LED2 = 0;
			LED3 = 0;
			LED4 = 0;
		}
		else if(pwr_time > (STARTTIME * 4 + VOLDISTOUCH))
		{
			CloseBootHandler();
			Voldisflag = 0;
			pwr_status = BOOT_STOP;
			pwr_time = 0;
		}
	}
}

/********************************
关机处理函数
功能：
	关机相关变量初始化
	
参数：
	无
	
返回值：
	无
**************************************/

void CloseBootHandler(void)
{
	SystemTime = 0;
	TIM2->CCER &=  0xffef;
	MoveMotorStatus = MOTORMOVESTOP;
	Ortate_Motor_Brate();
	BoardSt = NORMAL;
	warnlv = NOWARN;
	NowVol = 0;
//		NowCur = 0;
	NowTemp = 0;
	NowWp = 0;
	Speed = SPEED0;
	now_speed = 0;
	target_speed = 0;
	cpflag = 0;
	beepwarnontime = 0;
	yelflashtimes = 0;
	redflashtimes = 0;
	MotorMoveTime = 0;
	OrtateMotorLock = 0;
	OrtateMotorTime = 0;
	OrateMoveTime = 0;
	BeepIndTime = 0;
	memset(&condev,0,sizeof(condev));
	BEEP = 0;
	DeviceMode = INCH_MODE;
	CAN_Send_Msg(NULL, 0, MAIN_BOARD, CLOSE_BOOT);
	while(CAN_GetFlagStatus(CAN1, CAN_FLAG_RQCP0) == RESET);
	BoradVol = VOL_NONE;
	
	Led_Reset();
}

/********************************
电量显示函数
功能：
	BOOT_INIT状态下电量显示
	
参数：
	无
	
返回值：
	无
**************************************/
void Voldisplay(float vol)
{
	if(vol > VOLTAGE3)
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 1;
		LED4 = 1;
	}
	else if(vol <= VOLTAGE3 && vol > VOLTAGE2)
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 1;
		LED4 = 0;
	}
	else if(vol <= VOLTAGE2 && vol > VOLTAGE1)
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 0;
		LED4 = 0;
	}
	else if(vol <= VOLTAGE1 && vol > VOLTAGE0)
	{

		LED1 = 1;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
	}
	else
	{
		LED1 = 0;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
	}
}

/********************************
主控处理函数
功能：
	控制处理
	
参数：
	无
	
返回值：
	无
**************************************/
void Control_Handler(void)
{
	if(pwr_status == BOOT_RUN)
	{
		BootRunHandler();
	}
	else if(pwr_status == BOOT_INIT)
	{
		KeyShakeCheck();
		KeyHandler();
		BootInitHandler();
	}
	else
	{
		
		KeyShakeCheck();
		KeyHandler();
		Led_Reset();
//		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	}
}

/********************************
定时器3初始化
功能：
	主要用来做开关机按钮的定时
	
参数：
	无
	
返回值：
	无
**************************************/
#if 0
void TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStr;
	NVIC_InitTypeDef NVIC_Str;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	NVIC_Str.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Str.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Str);

	TIM_DeInit(TIM3);

	TIM_BaseStr.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStr.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStr.TIM_Period = STOP_TIME;     //两秒
	TIM_BaseStr.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseInit(TIM3, &TIM_BaseStr);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

//	TIM_Cmd(TIM3, ENABLE);
}
#endif
/********************************
时钟配置函数
功能：
	从停机模式下唤醒之后： 配置系统时钟允许HSE，和 pll 作为系统时钟。
	
参数：
	无
	
返回值：
	无
**************************************/
void SYSCLKConfig_STOP(void)
{
	ErrorStatus HSEStartUpStatus;
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{

		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}
}


