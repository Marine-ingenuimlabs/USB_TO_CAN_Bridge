#include "BridgeCAN_Lib.h"


extern COM_struct Bridge;

/*******************************************************************************
* Function Name  : CAN_Config
* Description    : Configure the CAN 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_Config()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
  /* Connect CAN pins to AF7 */
  GPIO_PinAFConfig(GPIOD, GPIO_Pin_0, GPIO_AF_7);
  GPIO_PinAFConfig(GPIOD, GPIO_Pin_1, GPIO_AF_7); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* NVIC configuration *******************************************************/
 /* NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x8;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;//CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  
   /* CAN Baudrate = 125 KBps (APB1 clocked at 42 Mhz = CANclock) */ // FOR stm32 f4
  /*CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
  CAN_InitStructure.CAN_Prescaler = 24;
  CAN_Init(CAN1, &CAN_InitStructure); */
  
  /* CAN Baudrate = 125 KBps (APB1 clocked at 36 MHz = CAN clock) */// for stm32 f3
  CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 16;
  CAN_Init(CAN1, &CAN_InitStructure);

  /* CAN filter init "FIFO0" *//*
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x6420;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x2461;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);*/
  
  /* CAN filter init "FIFO1" 
  CAN_FilterInitStructure.CAN_FilterNumber = 1;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x2460;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);*/
    
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  
  /* Transmit Structure preparation */
  /*TxMessage.StdId = 0x321;
  TxMessage.ExtId = 0x00;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.DLC = 1;*/
  
  /* Transmit Structure preparation 2*/
  Bridge.TxMessage1.StdId = 0x256;
  Bridge.TxMessage1.ExtId = 0x1FFFFF7;
  Bridge.TxMessage1.RTR = CAN_RTR_DATA;
  Bridge.TxMessage1.IDE = CAN_ID_STD;
  Bridge.TxMessage1.DLC = 7;
  Bridge.TxMessage1.Data[0]='I';
  Bridge.TxMessage1.Data[1]='N';
  Bridge.TxMessage1.Data[2]='I';
  Bridge.TxMessage1.Data[3]='T';
  Bridge.TxMessage1.Data[4]='_';
  Bridge.TxMessage1.Data[5]='O';
  Bridge.TxMessage1.Data[6]='K';
  

  /* Enable FIFO 0 message pending Interrupt */
 /* CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);*/
  
  /* Enable FIFO 1 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
  CAN_ITConfig(CAN1, CAN_IT_FF1, ENABLE);
  CAN_ITConfig(CAN1, CAN_IT_ERR, ENABLE);
}


/*******************************************************************************
* Function Name  : CAN_SaveMSG
* Description    : Save the CAN  message  into a message structre (to the pc ) 
* Input          : -src: CAN message 
*                  -dst: Message structure       
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t CAN_SaveMsg(CanRxMsg * src, Message * dst)
{
    if (src->IDE==CAN_ID_EXT  )
    { 
      if (IS_STRM_ID(src->ExtId))
      {
        dst->Mode = STRM_MODE;
        dst->Id = src->ExtId;
      }
      else if (IS_RQST_ID(src->ExtId)) {
        dst->Mode = RQST_MODE; 
        dst->Id = src->ExtId;
        }
    }
    else
    {
      if (IS_CMD_ID(src->StdId))
      {
        dst->Mode = CMD_MODE;
        dst->Id = src->StdId;
      }  
        else 
          return 0;
      
    }
    dst->Length=src->DLC;
    memcpy(dst->Data,src->Data,dst->Length);
    return 1;
}

/*******************************************************************************
* Function Name  : CAN_SendMsg.
* Descriptioan   : CRC16 computation for the entire message.
* Input          : Message to compte its CRC16: COM_Message ; 
* Output         : None.
* Return         : Message CRC value: Local_crc.
*******************************************************************************/
void CAN_SendMsg(Message* MsgToSend)
{ 
 
  if(MsgToSend->Mode==STRM_MODE)
  {
    Bridge.TxMessage1.IDE=CAN_ID_EXT;
    Bridge.TxMessage1.ExtId=MsgToSend->Id;
    Bridge.TxMessage1.RTR = CAN_RTR_DATA;
  }
  else
    if(MsgToSend->Mode==RQST_MODE)
    {
      Bridge.TxMessage1.IDE=CAN_ID_EXT;
      Bridge.TxMessage1.ExtId=MsgToSend->Id;
      Bridge.TxMessage1.RTR=CAN_RTR_REMOTE;
      
    }
    else
    {
      Bridge.TxMessage1.IDE=CAN_ID_STD;
      Bridge.TxMessage1.StdId=MsgToSend->Id;
      Bridge.TxMessage1.RTR = CAN_RTR_DATA;
    }
  Bridge.TxMessage1.DLC=MsgToSend->Length;
  memcpy(Bridge.TxMessage1.Data,MsgToSend->Data,MsgToSend->Length);
}
