
#ifndef __MAIN_CONTROL_H__
#define __MAIN_CONTROL_H__

#include "stm32f10x_pwr.h"
#include "stm32f10x_tim.h"
#include "config.h"

#define BOOT_RUN    2  //开机
#define BOOT_STOP   1  //停机
#define BOOT_INIT   0  //初始化

#define DEVINIT  0  //开机初始状态
#define DEVLOSE  1 //设备掉线
#define DEVCONN 2 //连接状态

#define STOP_TIME  2000  //长按关机时间  这里是2秒

#define MAX_CON_DEVNUM 4  //最大设备连接个数
#define MAX_HEART_TIME 6000 //心跳超时时间 4.5s

#define MOTORMOVERUN 1
#define MOTORMOVESTOP 0

#define MAX_MOVE_TIME 200  //电机一条命令最大转动时间 200ms

#define INCH_MODE 0 //点动模式
#define LINK_MODE 1 //联动模式

#define BEEP_ONTIME 250
#define BEEP_OFFTIME 50
#define BEEP_DELAY  250
#define BEEP_ONTIMES 2

#define RGB_SLOW_FLASH 750
#define RGB_NORMAL_FLASH 500
#define RGB_QUIK_FLASH 250

#define RGB_FLASH_TIMES 4 //最大闪烁次数

#define MAX_BIGCURRENTTIME 60000 //大电流报警时间60s

#define TEMP70  2.135 	//70度温度的电阻值
#define TEMP80  1.586   //80度温度的电阻值
#define TEMP90  1.236   //90度温度的电阻值

#define VOLCHANGEVAL  0.3    //电压变化差值 单位：V
#define TEMPCHANGEVAL 0.05   //温度变化差值 单位：千欧
#define CURCHANGEVAL  0.5    //电流变化差值 单位：A
#define WPCHANGEVAL   2      //功率变化差值 单位：W

#define BEEPINDMAXTIME 100  //蜂鸣器指示时间200ms

#define PAIR_MAX_TIME 10000  //配对最大时间10S
#define KEYPAIRTOUCH 2000 //按键按下配对时间2S

#define LIMIT_CURRENT_SPD SPEED4

#define ORTATEMOTORTIME 7000 //一次最多转动7秒

#define STARTTIME 250 //开机流水灯间隔

#define VOLDISTOUCH 300 //按键电量显示
#define VOLDISTIME  2000 //电量显示时间

//电池电压
#define VOLTAGE0  9
#define VOLTAGE1  10
#define VOLTAGE2  11
#define VOLTAGE3  12
#define VOLTAGE4 14.5
#define VOLTAGEMAX 18

#define SPEED_ADD   0x64 //速度加一档
#define SPEED_SUB   0x65 //速度减一档
#define SPEED_ONMAX 0x66 //速度切换，第一次收到时切换到最大档，第二次收到时切换到原始档位

#define CUR_NOISE 2.7  //电流噪声2.7A
#define CUR_BIAS  0.8  //电流误差参数
typedef struct
{
	u8 dev_id;     //控制设备id
	u8 status;	   //连接状态
//	u8 dev_mode;   //控制设备模式 联动或者点动
	u32 heart_time; //心跳时间
}SubDevStatus;

typedef struct
{
	u8 connum;                          //已连接设备数量
	SubDevStatus dev[MAX_CON_DEVNUM];   //连接设备列表
}ConnectDev;

typedef enum
{
	VOL_FULL = 0,
	VOL_LEVEL3 = 1,
	VOL_LEVEL2 = 2,
	VOL_LEVEL1 = 3,
	VOL_NONE = 4,
}VolStatus;

typedef enum
{
	NORMAL = 0,
	ST_PAIR,
	ST_CANCELPAIR,
#ifndef DR_UPDATE
	ORTATE_FAULT,
#endif
	VOL_FAULT,
	HIGH_VOL_FAULT,
	TEMP_FAULT,
//	CURRENT_FAULT60,
//	CURRENT_FAULT70,
	WORKPOWER_FAULT,
	CONNECT_FAULT,
	REEDKEY_FAULT,
	MOTOR_FAULT,
}BoardStatus;

typedef enum
{
	NOWARN = 0,
	PAIRWARN,
	CANCELPAIRWARN,
	ORTATEWARN,
	VOLWARN,
	TEMPWARN,
//	CURRENT60WARN,
//	CURRENT70WARN,
	WORKPOWERWARN,
	CONNECTWARN,
	REEDKEYWARN,
	MOTORWARN,
}WarningLevel;

extern __IO u8 pwr_status;
extern CommandBuf rxbuf;
extern __IO u16 pwr_time;
extern u8 Speed;
extern BoardStatus BoardSt;
extern __IO u16 StartTime;
extern u8 Voldisflag;


void TIM3_Init(void);
void SYSCLKConfig_STOP(void);
void Control_Handler(void);
void HeartBeatCounter(void);
void DeviceStatusInit(void);
void MotorMoveStart(void);
void MotorMoveStop(void);
void MotorMoveSpeedSet(void);
u8 SearchDevice(u8 devid);
void ReflashHeartBeat(u8 devid);
void HeartBeatHandler(u8 devid);
void ConnectHandler(CommandData* dev);
void OrtateMotorControl(CommandData* dev);
void SpeedControlHandler(CommandData* dev);
void MotorMoveControlHandler(CommandData* dev);
void ModeChangeHandler(CommandData* dev);
void MotorMoveCounter(void);
void ADCHandler(void);
void VoltageHandler(float vol);
void FaultHandler(void);
void WarningTimeCounter(void);
void WarningHandler(void);
void ConnectCheck(void);
void OrtateFaultCheck(void);
void WorkPowerHandler(float cur,float vol);
void KeyHandler(void);
void QuitPair(void);
void PairAckHandler(CommandData* dev);
void CloseBootCheck(void);
void CloseBootHandler(void);
void Voldisplay(float vol);
void Volreflash(void);
void SpeedChange(void);
void TempCheck(void);
void VolCheck(void);

#endif

