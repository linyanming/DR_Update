
#ifndef __CAN_H__
#define __CAN_H__

#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "config.h"

void CAN_Config(void);
u8 CAN_Send_Msg(u8 *msg,u8 len,Device_ID dev_id,Device_CMD dev_cmd);
u8 CAN_Receive_Msg(CommandData *msg);


#endif
