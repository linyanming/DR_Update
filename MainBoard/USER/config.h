
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

//#define NULL ((void *)0)

//#define USART_DEBUG   //串口调试宏定义s

#ifdef USART_DEBUG
//#define DEBUGMSG(s,...) printf(s,##__VA_ARGS__);printf(",%d,%s\r\n",__LINE__,__FILE__)
#define DEBUGMSG(s,...) printf("File: "__FILE__", Line: %05d: "s"\n",__LINE__,##__VA_ARGS__)

#else
#define DEBUGMSG(s,...)	
#endif

#define MAX_DATA_LEN  8  //数据帧最大数据长度
#define RESERVED_DATA 3  //标识符保留位数据

#define DR_UPDATE

//设备ID定义   ，最多定义16个ID
typedef enum
{
	MAIN_BOARD = 0,    //主控器
	FOOT_BOARD = 1,    //脚控器
	REMOTE_BOARD = 2,  //遥控
}Device_ID;

//命令定义  最多定义32条命令
typedef enum
{
	HEARTBEAT = 0,    //心跳
	LEFT_TURN = 1,    //左转
	RIGHT_TURN = 2,   //右转
	ORTATE_STOP = 3,  //转动停止
	START_MOVE = 4,   //前进
	RETREAT = 5,      //后退
	STOP_MOVE = 6,    //停止移动
	SPEED_CNTR = 7,   //调速
	CONNECT = 8,	  //连接命令
	CONNECT_ACK = 9, //连接应答
	MODE_CHANGE = 10,    //模式切换
	MOTOR_CHANGE = 11,    //控制状态反馈
	START_BOOT = 12,  //开机
	CLOSE_BOOT = 13,  //关机
	PAIRING = 14,     //配对
	CANCEL_PAIR = 15, //取消配对
	PAIR_ACK = 16,    //配对应答
}Device_CMD;

//命令数据结构体
typedef struct
{
	u8 dev_id;     //设备ID
	u8 dev_cmd;    //设备命令
	u8 datalen;
	u8 data[MAX_DATA_LEN];
}CommandData;

typedef struct
{
	u8 Rxflag;        //缓存中是否有数据在处理
	CommandData cmd;
}CommandBuf;

#endif
