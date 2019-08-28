
#include "can.h"
#include "config.h"

/************************************************
CAN初始化函数 初始化CAN波特率为500kbps

波特率计算公式：CAN总线波特率=APB1时钟/((BS1+BS2+SJW)*分频系数)

参数：
	无

返回值：
	无
*************************************************/
void CAN_Config(void)
{
	GPIO_InitTypeDef GPIO_Str;
	CAN_InitTypeDef CAN_Str;
	CAN_FilterInitTypeDef CAN_FilterStr;
	NVIC_InitTypeDef NVIC_Str;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_Str.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Str.GPIO_Pin = GPIO_Pin_9;
	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_Str);
	
	GPIO_Str.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Str.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_Str);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

	CAN_Str.CAN_ABOM = DISABLE;  //软件自动离线管理
	CAN_Str.CAN_AWUM = DISABLE;  //睡眠模式通过软件唤醒
	CAN_Str.CAN_BS1 = CAN_BS1_9tq; //定义采样点位置
	CAN_Str.CAN_BS2 = CAN_BS2_8tq; //定义发送点位置
	CAN_Str.CAN_SJW = CAN_SJW_1tq; //
	CAN_Str.CAN_Prescaler = 8;    //时钟分频系数，CAN总线波特率=APB1时钟/((BS1+BS2+SJW)*分频系数)                 ，在这里为 250kbps
	CAN_Str.CAN_Mode = CAN_Mode_Normal;
	CAN_Str.CAN_NART = ENABLE;   //自动重传功能
	CAN_Str.CAN_RFLM = DISABLE;   //是否使用FIFO
	CAN_Str.CAN_TTCM = DISABLE;  //时间触发功能
	CAN_Str.CAN_TXFP = DISABLE;  //使能时会按照消息进入邮箱的先后顺序发送
	CAN_Init(CAN1, &CAN_Str);

	CAN_FilterStr.CAN_FilterActivation = ENABLE;
	CAN_FilterStr.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterStr.CAN_FilterIdHigh = 0x0000;
	CAN_FilterStr.CAN_FilterIdLow = 0x0000;
	CAN_FilterStr.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterStr.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterStr.CAN_FilterMode = CAN_FilterMode_IdMask; //标识符列表模式
	CAN_FilterStr.CAN_FilterNumber = 0;  //选用筛选器0
	CAN_FilterStr.CAN_FilterScale = CAN_FilterScale_32bit;  //工作在32位模式
	CAN_FilterInit(&CAN_FilterStr);

	NVIC_Str.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Str.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_Str);

/*	NVIC_Str.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Str.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Str.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_Str);*/
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
//	CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE);
//	CAN_ITConfig(CAN1, CAN_IT_ERR, ENABLE);
	DEBUGMSG("CAN init");
	return;
}

/************************************************
CAN数据发送函数

参数：
	msg：消息内容
	led：消息长度
	dev_id：设备ID
	dev_cmd：设备命令

返回值：
	0：发送成功
	1：发送失败
*************************************************/
u8 CAN_Send_Msg(u8 *msg,u8 len,Device_ID dev_id,Device_CMD dev_cmd)
{
	CanTxMsg TxMsg;
	u16 i = 0;
	u8 mbox;

	if(len > MAX_DATA_LEN)
	{
		DEBUGMSG("CAN Send fail len > max");
		return 1;
	}
	
//	TxMsg.ExtId = 0x12;
	TxMsg.StdId = ((dev_id << 7) | (dev_cmd << 2) | RESERVED_DATA) & 0x7ff;
	TxMsg.DLC = len;
	TxMsg.RTR = 0;
	TxMsg.IDE = 0;
	for(i = 0;i < len;i++)
		TxMsg.Data[i] = msg[i];
	
	mbox = CAN_Transmit(CAN1, &TxMsg);
	switch(mbox)
	{
		case CAN_TxStatus_NoMailBox:
			while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_NoMailBox) && (i < 0xfff)) 
				i++;
			if(i >= 0xfff)
			{
				DEBUGMSG("CAN Send fail CAN_TxStatus_NoMailBox");
				return 1;
			}
			break;
		default:
			break;
	}
	DEBUGMSG("CAN_Send_Msg %d %d",dev_id,dev_cmd);
	return 0;
}

/************************************************
CAN数据接收函数

参数：
	msg：数据接收缓存结构体

返回值：
	0：接收成功
	1：接收失败
*************************************************/
u8 CAN_Receive_Msg(CommandData *msg)
{
	CanRxMsg RxMsg;
	u8 i;
	
	if(CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
	{
		DEBUGMSG("CAN_MessagePending == 0");
		return 1;
	}
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMsg);
	msg->dev_id = (RxMsg.StdId >> 7) & 0x0f;
	msg->dev_cmd = (RxMsg.StdId >> 2) & 0x1f;
	msg->datalen = RxMsg.DLC;
	for(i = 0;i < RxMsg.DLC;i++)
		msg->data[i] = RxMsg.Data[i];

	DEBUGMSG("CAN_Receive_Msg");
	return 0;
}


