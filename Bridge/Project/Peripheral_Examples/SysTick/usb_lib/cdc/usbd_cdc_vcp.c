/**
 ******************************************************************************
 * @file    usbd_cdc_vcp.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    22-July-2011
 * @brief   Generic media access Layer.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_vcp.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_usart.h"
#include "BRIDGE_Config.h"

/* Private variables ---------------------------------------------------------*/
LINE_CODING linecoding = {
		115200, /* baud rate*/
		0x00, /* stop bits-1*/
		0x00, /* parity - none*/
		0x08 /* nb. of bits 8*/
};
    
uint8_t Receive_Counter;
COM_struct Bridge;
USART_InitTypeDef USART_InitStructure;

/* These are external variables imported from CDC core to be used for IN 
 transfer management. */
extern uint8_t APP_Rx_Buffer[]; /* Write CDC received data in this buffer.
 These data will be sent over USB IN endpoint
 in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in; /* Increment this pointer or roll it back to
 start address when writing received data
 in the buffer APP_Rx_Buffer. */

/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len);

CDC_IF_Prop_TypeDef VCP_fops = { VCP_Init, VCP_DeInit, VCP_Ctrl, VCP_DataTx,
		VCP_DataRx };

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  VCP_Init
 *         Initializes the Media on the STM32
 * @param  None
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static uint16_t VCP_Init(void) {
	return USBD_OK;
}

/**
 * @brief  VCP_DeInit
 *         DeInitializes the Media on the STM32
 * @param  None
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static uint16_t VCP_DeInit(void) {
	return USBD_OK;
}

/**
 * @brief  VCP_Ctrl
 *         Manage the CDC class requests
 * @param  Cmd: Command code
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len) {
	switch (Cmd) {
	case SEND_ENCAPSULATED_COMMAND:
		/* Not  needed for this driver */
		break;

	case GET_ENCAPSULATED_RESPONSE:
		/* Not  needed for this driver */
		break;

	case SET_COMM_FEATURE:
		/* Not  needed for this driver */
		break;

	case GET_COMM_FEATURE:
		/* Not  needed for this driver */
		break;

	case CLEAR_COMM_FEATURE:
		/* Not  needed for this driver */
		break;

	case SET_LINE_CODING:
		/* Not  needed for this driver */
		break;

	case GET_LINE_CODING:
		Buf[0] = (uint8_t) (linecoding.bitrate);
		Buf[1] = (uint8_t) (linecoding.bitrate >> 8);
		Buf[2] = (uint8_t) (linecoding.bitrate >> 16);
		Buf[3] = (uint8_t) (linecoding.bitrate >> 24);
		Buf[4] = linecoding.format;
		Buf[5] = linecoding.paritytype;
		Buf[6] = linecoding.datatype;
		break;

	case SET_CONTROL_LINE_STATE:
		/* Not  needed for this driver */
		break;

	case SEND_BREAK:
		/* Not  needed for this driver */
		break;

	default:
		break;
	}

	return USBD_OK;
}

/**
 * @brief  putchar
 *         Sends one char over the USB serial link.
 * @param  buf: char to be sent
 * @retval none
 */

void VCP_put_char(uint8_t buf) {
	VCP_DataTx(&buf, 1);
}

void VCP_send_str(uint8_t* buf) {
	uint32_t i = 0;
	while (*(buf + i)) {
		i++;
	}
	VCP_DataTx(buf, i);
}

void VCP_send_buffer(uint8_t* buf, int len) {
	VCP_DataTx(buf, len);
}

/**
 * @brief  VCP_DataTx
 *         CDC received data to be send over USB IN endpoint are managed in
 *         this function.
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
 */
static uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len) {
	uint32_t i = 0;
	while (i < Len) {
		APP_Rx_Buffer[APP_Rx_ptr_in] = *(Buf + i);
		APP_Rx_ptr_in++;
		i++;
		/* To avoid buffer overflow */
		if (APP_Rx_ptr_in == APP_RX_DATA_SIZE) {
			APP_Rx_ptr_in = 0;
		}
	}

	return USBD_OK;
}

/**
 * @brief  VCP_DataRx
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         until exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
 */

#define APP_TX_BUF_SIZE 4096
uint8_t APP_Tx_Buffer[APP_TX_BUF_SIZE];
uint32_t APP_tx_ptr_head;
uint32_t APP_tx_ptr_tail;

static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len) {
	uint32_t i;
        APP_tx_ptr_tail=APP_tx_ptr_head + Len;
	for (i = 0; i < Len; i++) {
		APP_Tx_Buffer[APP_tx_ptr_head] = *(Buf + i);
                CDC_Receive_DATA(APP_Tx_Buffer[APP_tx_ptr_head]);
                //printf(" %d, %d \r\n",APP_tx_ptr_tail,APP_tx_ptr_head);
		APP_tx_ptr_head++; 
		if (APP_tx_ptr_head == APP_TX_BUF_SIZE)
			APP_tx_ptr_head = 0;

		if (APP_tx_ptr_head == APP_tx_ptr_tail)
			return USBD_FAIL;
	}

	return USBD_OK;
}

int VCP_get_char(uint8_t *buf) {
	if (APP_tx_ptr_head == APP_tx_ptr_tail)
		return 0;

	*buf = APP_Tx_Buffer[APP_tx_ptr_tail];
	APP_tx_ptr_tail++;
	if (APP_tx_ptr_tail == APP_TX_BUF_SIZE)
		APP_tx_ptr_tail = 0;

	return 1;
}

int VCP_get_string(uint8_t *buf) {
	if (APP_tx_ptr_head == APP_tx_ptr_tail)
		return 0;

	while (!APP_Tx_Buffer[APP_tx_ptr_tail]
			|| APP_Tx_Buffer[APP_tx_ptr_tail] == '\n'
			|| APP_Tx_Buffer[APP_tx_ptr_tail] == '\r') {
		APP_tx_ptr_tail++;
		if (APP_tx_ptr_tail == APP_TX_BUF_SIZE)
			APP_tx_ptr_tail = 0;
		if (APP_tx_ptr_head == APP_tx_ptr_tail)
			return 0;
	}

	int i = 0;
	do {
		*(buf + i) = APP_Tx_Buffer[i + APP_tx_ptr_tail];
		i++;

		if ((APP_tx_ptr_tail + i) == APP_TX_BUF_SIZE)
			i = -APP_tx_ptr_tail;
		if (APP_tx_ptr_head == (APP_tx_ptr_tail + i))
			return 0;

	} while (APP_Tx_Buffer[APP_tx_ptr_tail + i]
			&& APP_Tx_Buffer[APP_tx_ptr_tail + i] != '\n'
			&& APP_Tx_Buffer[APP_tx_ptr_tail + i] != '\r');

	*(buf + i) = 0;
	APP_tx_ptr_tail += i;
	if (APP_tx_ptr_tail >= APP_TX_BUF_SIZE)
		APP_tx_ptr_tail -= APP_TX_BUF_SIZE;
	return i;
}

/**
 * @brief  EVAL_COM_IRQHandler
 *
 * @param  None.
 * @retval None.
 */
void EVAL_COM_IRQHandler(void) {

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
  if(Bridge.COM_Status!=COM_ReadyToProcess){
  switch (Bridge.COM_Status)
  { 
    /*Header reception*/  
    case COM_WaitingforHeader:
      if(Received_Char==RQST_HHEADER || Received_Char==CMD_HHEADER)
      { 
        GPIO_SetBits(GPIOD, GPIO_Pin_13);
        Bridge.COM_State=ChangeState_level_Busier(Bridge.COM_State);
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
       Receive_Counter=0;
       Bridge.COM_State=ChangeState_level_Busier(Bridge.COM_State);
      }
      else 
      {
        //strcpy((char*)APP_Tx_Buffer,"");
        Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State);  
        Bridge.COM_Status=COM_WaitingforHeader;
        
      }
      break;
      
    /* Special Message reception*/ 
    case COM_WaitingforSPCL:
      Receive_Counter++;
      if(Receive_Counter==1){
        if(Received_Char!=ACK_MESSAGE[0])
        {
          Bridge.COM_Status=COM_WaitingforID;  
        }else 
        {
          Bridge.COM_Status=COM_WaitingforHeader;
          Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State);
        }
          
      }else{  
        Bridge.COM_Acknowledgement.COM_ACK = Received_Char;
        Bridge.COM_Acknowledgement.State = ACK_RECEIVED;
        Bridge.COM_Status=COM_WaitingforHeader;
        Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State); 
      }
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
      {   if (Received_Char == 0 && (Bridge.Reception_Buffer.Mode==0xFF-RQST_HHEADER))
          {
             Bridge.COM_Status=COM_WaitingforCRC;
          }
      else{
    
          Bridge.Reception_Buffer.Length=Received_Char;
          Bridge.COM_Status=COM_WaitingforData;
          Bridge.crc = crc16(Bridge.crc,Received_Char);
            }
      }
      else {
        Bridge.COM_Status=COM_WaitingforHeader;
        COM_SendSpecial((unsigned char *)&NACK_MESSAGE);
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
        { //STM_EVAL_LEDToggle(LED3);
          COM_SendSpecial((unsigned char *)&NACK_MESSAGE);
          Bridge.COM_Status=COM_WaitingforHeader;
          Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State); 
        }
        else 
        { 
           /* Turn On LED4 */
          //STM_EVAL_LEDToggle(LED4);
          COM_SendSpecial((unsigned char *)ACK_MESSAGE);
          Bridge.COM_Status=COM_ReadyToProcess;
          GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
        }
      }  
    break;
    
  default:
    break;
  }
  }
  
  return 1 ;
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
{ uint8_t Header= SPCL_HHEADER;
  VCP_send_buffer (&Header,1);
  VCP_send_buffer ((unsigned char*)Id,2);
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
