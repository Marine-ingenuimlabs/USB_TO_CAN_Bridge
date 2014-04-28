/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "stm32f30x_tim.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

    
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
    static const uint16_t  CRC_TABLE[256] ={
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
ErrorStatus HSEStartUpCOM_Status;
EXTI_InitTypeDef EXTI_InitStructure;
extern __IO uint32_t packet_sent;
extern __IO uint8_t Send_Buffer[VIRTUAL_COM_PORT_DATA_SIZE] ;
extern __IO  uint32_t packet_receive;
extern __IO uint8_t Receive_length;
COM_struct Bridge;
uint8_t Receive_Buffer[16];
uint32_t Send_length;
uint8_t Receive_Counter;
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
#if !defined(STM32L1XX_MD) && !defined(STM32L1XX_HD) && !defined(STM32L1XX_MD_PLUS)
  GPIO_InitTypeDef GPIO_InitStructure;
#endif /* STM32L1XX_MD && STM32L1XX_XD */  

#if defined(USB_USE_EXTERNAL_PULLUP)
  GPIO_InitTypeDef  GPIO_InitStructure;
#endif /* USB_USE_EXTERNAL_PULLUP */ 
  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */   
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS) || defined(STM32F37X) || defined(STM32F30X)
  /* Enable the SYSCFG module clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif /* STM32L1XX_XD */ 
   
#if !defined(STM32L1XX_MD) && !defined(STM32L1XX_HD) && !defined(STM32L1XX_MD_PLUS) && !defined(STM32F37X) && !defined(STM32F30X)
  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up pin */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32L1XX_MD && STM32L1XX_XD */
   
#if defined(USB_USE_EXTERNAL_PULLUP)
  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);

  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);  
#endif /* USB_USE_EXTERNAL_PULLUP */ 
  
#if defined(STM32F37X) || defined(STM32F30X)
  
  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);
  
  /*Set PA11,12 as IN - USB_DM,DP*/ 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*SET PA11,12 for USB: USB_DM,DP*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_14);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_14);
  
  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32F37X  && STM32F30X)*/
 
   /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD) || defined(STM32L1XX_MD_PLUS) 
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  
#else 
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
    /*Enable SystemCoreClock*/
  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
#if defined(STM32L1XX_MD)|| defined(STM32L1XX_HD) || defined(STM32L1XX_MD_PLUS)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#elif defined(STM32F37X)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#else
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);   
#endif


}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : COM_Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#if defined(STM32L1XX_MD) || defined (STM32L1XX_HD)|| (STM32L1XX_MD_PLUS)
  if (NewState != DISABLE)
  {
    STM32L15_USB_CONNECT;
  }
  else
  {
    STM32L15_USB_DISCONNECT;
  }  
  
#else /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;
 
  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : Send DATA .
* Description    : send the data received from the STM32 to the PC through USB  
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t CDC_Send_DATA (uint8_t *ptrBuffer, uint8_t Send_length)
{
  /*if max buffer is Not reached*/
  if(Send_length < VIRTUAL_COM_PORT_DATA_SIZE)     
  {
    /*Sent flag*/
    packet_sent = 0;
    /* send  packet to PMA*/
    UserToPMABufferCopy((unsigned char*)ptrBuffer, ENDP1_TXADDR, Send_length);
    SetEPTxCount(ENDP1, Send_length);
    SetEPTxValid(ENDP1);
  }
  else
  {
    return 0;
  } 
  return 1;
}

/*******************************************************************************
* Function Name  : ChangeState_level_Freer
* Description    : level up  the state  
* Input          : 
*                 - state: The state to promote or the demote 
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t ChangeState_level_Freer(uint8_t State)
{ 
  if (State== COM_IDLE || State== CAN_IDLE)
    //printf("The state is going out of range (down): %d", State);
  if(State == CAN_MSGINQUEUE || State == CAN_PROC_MSGINQUEUE || State == COM_MSGINQUEUE || State == COM_PROC_MSGINQUEUE )
    {
    State--;
    }
    State--;
  return State;
}
/*******************************************************************************
* Function Name  : ChangeState_level_Freer
* Description    : level up  the state  
* Input          : 
*                 - state: The state to promote or the demote 
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t ChangeState_level_Busier(uint8_t State)
{ if (State== CAN_PROC_MSGINQUEUE || State== COM_PROC_MSGINQUEUE)
    {
    //printf("The state is going out of range (up): %d", State);
    }
  State++;
  return State;

}
/*******************************************************************************
* Function Name  : crc16.
* Descriptioan    : CRC16 computation.
* Input          : Current CRC:crcval ; Current received character:newchar.
* Output         : None.
* Return         : The new computed CRC value.
*******************************************************************************/
uint16_t crc16(uint16_t crcval, uint8_t newchar)
{
  crcval = (uint16_t)(  (uint16_t)(crcval >> 8) ^ CRC_TABLE[(crcval ^ newchar) & 0x00ff]);
  return crcval;
}

/*******************************************************************************
* Function Name  : Bridge_Init.
* Descriptioan   : initialize the Bridge structure.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Bridge_Init(COM_struct* USBtoCAN)
{
  USBtoCAN->COM_Status=COM_WaitingforHeader;
  USBtoCAN->COM_Err.ErrorId=0;
  USBtoCAN->CAN_Err.ErrorId=0;
  USBtoCAN->crc_pc=0;
  USBtoCAN->Reception_Buffer.Mode=0;
  USBtoCAN->CAN_State=CAN_IDLE;
  USBtoCAN->COM_State=COM_IDLE;
  USBtoCAN->COM_Acknowledgement.COM_ACK=NO_ACK;
  USBtoCAN->COM_Acknowledgement.Nb_Check=0;
  USBtoCAN->CAN_Acknowledgement.Nb_Tries=0;
  USBtoCAN->CAN_Acknowledgement.State=ACK_IDLE;
}

/*******************************************************************************
* Function Name  : COM_ClearUsbBuffer
* Description    : Clear the buffer of USB reception
* Input          : -src: Message structure
*                  -dst: Message structure       
* Output         : None
* Return         : None
*******************************************************************************/
void COM_ClearUsbBuffer(char* Buffer){
  strcpy((char*)Receive_Buffer, "");
}

/*******************************************************************************
* Function Name  : COM_SendSpecial
* Description    : Send special message to the user USB
* Input          : 
                    - Id: message identifier 
* Output         : None
* Return         : None
*******************************************************************************/
void COM_SendSpecial(uint8_t* Id )
{
  CDC_Send_DATA ((unsigned char*)SPCL_HHEADER,1);
  CDC_Send_DATA ((unsigned char*)Id,strlen((char*)Id));
}

/*******************************************************************************
* Function Name  : Receive DATA .
* Description    : receive the data from the PC to STM32 and send it through USB
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t CDC_Receive_DATA(uint8_t Received_Char)
{ 
  /*Receive flag*/
  switch (Bridge.COM_Status)
  {
    /*Header reception*/  
    case COM_WaitingforHeader:
      if(Received_Char==RQST_HHEADER || Received_Char==CMD_HHEADER)
      { //Bridge.COM_State++;
        //printf("%d",Bridge.COM_State);
        Bridge.COM_State=ChangeState_level_Busier(Bridge.COM_State);
        //printf("%d",Bridge.COM_State);
        Bridge.Reception_Buffer.Mode= 0xFF-Received_Char;
        Bridge.COM_Status=COM_WaitingforID;
        //Bridge.New_Message=1;
        Receive_Counter=0;
        Bridge.crc=0xFFFF;
        Bridge.crc = crc16(Bridge.crc,Received_Char);
      } 
      
      else if (Received_Char == SPCL_HHEADER && Bridge.COM_Acknowledgement.State==ACK_WAITING )
      {        
       Bridge.COM_Status=COM_WaitingforSPCL; 
      }
      else 
      {
        strcpy((char*)Receive_Buffer,"");
        Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State);  
      }
      break;
      
    /* Special Message reception*/ 
    case COM_WaitingforSPCL:
     Bridge.COM_Acknowledgement.COM_ACK = Received_Char;
        Bridge.COM_Acknowledgement.State = ACK_RECEIVED;
        Bridge.COM_Status=COM_WaitingforHeader;
        Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State); 
        break;
        
    /*Message Identifier reception*/
    case COM_WaitingforID:
        Bridge.Reception_Buffer.Id= Received_Char; 
        Bridge.COM_Status=COM_WaitingforLength;
        Bridge.crc = crc16(Bridge.crc,Received_Char);
      break;

    /*Data length integer */
    case COM_WaitingforLength:
      if(Received_Char <=8)
      {
          Bridge.Reception_Buffer.Length=Received_Char;
          Bridge.COM_Status=COM_WaitingforData;
          Bridge.crc = crc16(Bridge.crc,Received_Char);
      }
      else {
        Bridge.COM_Status=COM_WaitingforHeader;
        Bridge.COM_Err.ErrorId=COM_ERRLENGTH;
        Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State); 
      }
    break;
    /*Data reception*/
    case COM_WaitingforData:
      if(Receive_Counter < Bridge.Reception_Buffer.Length-1)
      {
         Bridge.Reception_Buffer.Data[Receive_Counter]= Received_Char;
         Bridge.crc = crc16(Bridge.crc,Received_Char);
         Receive_Counter++;
      }
      else 
        if(Receive_Counter==Bridge.Reception_Buffer.Length-1)
        {
          Bridge.Reception_Buffer.Data[Receive_Counter]= Received_Char;
          Bridge.crc = crc16(Bridge.crc,Received_Char);
          Bridge.COM_Status=COM_WaitingforCRC;
          Receive_Counter=0;
        }
    break;
    /*Checksum codded on 2 bytes*/
    case COM_WaitingforCRC:
      if(!Receive_Counter)
      {
        Bridge.crc_pc=Received_Char<<8; 
        Receive_Counter++;
      }
      else
      {
        Bridge.crc_pc=(uint16_t)(Bridge.crc_pc|Received_Char); 
        if(Bridge.crc!=Bridge.crc_pc ) //case the received message has a wrong checksum 
        { STM_EVAL_LEDToggle(LED3);
          COM_SendSpecial((unsigned char *)&NACK_MESSAGE);
          Bridge.COM_Status=COM_WaitingforHeader;
          Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State); 
        }
        else 
        { 
           /* Turn On LED4 */
          STM_EVAL_LEDToggle(LED4);
          COM_SendSpecial((unsigned char *)&ACK_MESSAGE);
          Bridge.COM_Status=COM_ReadyToProcess;
        }
      }  
    break;
    
  default:
    break;
  }
  return 1 ;
}

void LED_Init()
{
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  STM_EVAL_LEDInit(LED7);
  STM_EVAL_LEDInit(LED8);
  STM_EVAL_LEDInit(LED9);
  STM_EVAL_LEDInit(LED10);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
