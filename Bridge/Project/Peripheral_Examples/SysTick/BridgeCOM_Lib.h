#include "main.h"

void COM_SendMsg(Message * MsgToSend);
void COM_SaveMsg(Message * src,Message * dst);
uint16_t COM_CRC16(Message* COM_Message);
void COM_SendSpecial(uint8_t* Id );
