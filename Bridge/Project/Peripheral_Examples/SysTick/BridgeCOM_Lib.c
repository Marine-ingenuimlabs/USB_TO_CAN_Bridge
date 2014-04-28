#include "BridgeCOM_Lib.h"


extern COM_struct Bridge;
union_struct mybyteconvertor;
/*******************************************************************************
* Function Name  : COM_SendMessage
* Description    : Send message to the user USB  
* Input          : Message structure to send 
* Output         : None
* Return         : None
*******************************************************************************/
void COM_SendMsg(Message * MsgToSend)
{ uint8_t trame[13];
  trame[0]=0xFF-(MsgToSend->Mode);
  trame[1]= MsgToSend->Id;
  trame[2]=MsgToSend->Length;               
  mybyteconvertor.x16[0] = COM_CRC16(MsgToSend);
  memcpy(&trame[3],MsgToSend->Data,MsgToSend->Length);
  trame[3+MsgToSend->Length]= mybyteconvertor.x8[1];
  trame[3+MsgToSend->Length+1]= mybyteconvertor.x8[0];
  VCP_send_buffer (trame,5+MsgToSend->Length);
}

/*******************************************************************************
* Function Name  : COM_SaveMsg
* Description    : Save message structure (from the pc) 
* Input          : -src: Message structure
*                  -dst: Message structure       
* Output         : None
* Return         : None
*******************************************************************************/
void COM_SaveMsg(Message * src,Message * dst)
{ 
  for(int i=0;i<8;i++)
  dst->Data[i]=src->Data[i];
  dst->Mode=src->Mode;
  dst->Id=src->Id;
  dst->Length=src->Length; 
}

/*******************************************************************************
* Function Name  : COM_CRC16.
* Descriptioan   : CRC16 computation for the entire message.
* Input          : Message to compte its CRC16: COM_Message ; 
* Output         : None.
* Return         : Message CRC value: Local_crc.
*******************************************************************************/
uint16_t COM_CRC16(Message* COM_Message)
{ uint16_t Local_crc;
  Local_crc=crc16(0xFFFF,0xFF-COM_Message->Mode);
  Local_crc=crc16(Local_crc,COM_Message->Id);
  Local_crc=crc16(Local_crc,COM_Message->Length);
  for(int i=0;i<COM_Message->Length;i++)
    Local_crc=crc16(Local_crc,COM_Message->Data[i]);  
  return Local_crc;
}