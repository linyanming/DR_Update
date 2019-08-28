

#include "iwdg.h"


/************************************************
看门狗初始化函数
功能：
	看门狗初始化       喂狗时间计算：t=(4*2^pre*reload)/40
	这里为1280ms
	
参数：
	无

返回值：
	无
*************************************************/

void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //1、开启寄存器访问

	IWDG_SetPrescaler(IWDG_Prescaler_32);  //2、设置预分配值         喂狗时间计算公式T = 4*2^PRE/40*RLR(ms)

	IWDG_SetReload(0xfff); //3、设置重载值最大值0xfff

	IWDG_ReloadCounter(); //4、重载计数器值

	IWDG_Enable(); //5、使能独立看门狗

	DEBUGMSG("IWDG_Init");
}

/************************************************
喂狗
功能：
	重置看门狗计数器
	
参数：
	无

返回值：
	无
*************************************************/

void IWDG_FeedDog(void)
{
	IWDG_ReloadCounter();
	DEBUGMSG("IWDG_FeedDog");
}
