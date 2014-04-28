/**
  ******************************************************************************
  * @file    BridgeCAN_Lib.h
  * @author  Rami Hadj Taïeb 
  * @version V4.0.0
  * @date    07-March-2014
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  */
#include "main.h"
/* Exported functions ------------------------------------------------------- */
void CAN_Config();
uint8_t CAN_SaveMsg(CanRxMsg * src, Message * dst);
void CAN_SendMsg(Message* MsgToSend);
/* External variables --------------------------------------------------------*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/