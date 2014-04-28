/**
  ******************************************************************************
  * @file    hw_config.h
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


/* Define to prevent recursive inclusion -------------------------------------*/


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"

/* Exported types ------------------------------------------------------------*/
typedef struct 
{ uint8_t Data[8];
  uint8_t Length;
  uint8_t Id;
  uint8_t Mode;
}Message;

typedef struct 
{ Message CDC_FromPC_Buffer;
  Message CDC_ToPC_Buffer;
}LASTSAVED_struct;

typedef struct 
{ 
  uint8_t COM_ACK;
  uint8_t Nb_Check; //
  uint8_t Nb_Tries; //The sending repetetion number without reception of a valid ACK
  uint8_t State;// Idle / received /waiting
}ACK_struct;
typedef struct 
{
  uint8_t ErrorId;
  uint8_t ErrorData;
} ERROR_Struct;
typedef struct 
{ 
  /*serial variables*/
  uint8_t COM_State;
  ERROR_Struct COM_Err;
  uint16_t crc;
  uint16_t crc_pc;
  uint8_t COM_Status;
  Message Reception_Buffer;
  LASTSAVED_struct COM_Lastmessage; // Last input and output   
  ACK_struct COM_Acknowledgement; // CDC acknowledegement structure 
  ACK_struct CAN_Acknowledgement;
  
  /*CAN BUS variables*/
  uint8_t CAN_State; // specify the state of the CAN queue 
  ERROR_Struct CAN_Err;
  CanRxMsg RxMessage1; 
  CanTxMsg TxMessage1; //  CAN message instance used as auxiliary message to send message
}COM_struct;

typedef union 
{
  uint32_t x32;
  uint16_t x16[2];
  uint8_t  x8[4]; 
}union_struct;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/*#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040
#define LED_ON                0xF0
#define LED_OFF               0xFF*/

    /* Different communication's modes id */
#define STRM_MODE  1// streaaming mode (from)
#define RQST_MODE  2 // Request mode (Control station  down to ROV and then returns the demanded data)
#define CMD_MODE   3 // Commande mode (changing variable of the ROV from the control station )

    /* Different headers of communication's modes */ 
#define STRM_HHEADER 0xFE
#define RQST_HHEADER 0xFD
#define CMD_HHEADER  0xFC
#define SPCL_HHEADER 0xFB
    
    /* Different status of the reception*/
#define COM_WaitingforHeader   0
#define COM_WaitingforID       1
#define COM_WaitingforLength   2
#define COM_WaitingforData     3
#define COM_WaitingforCRC      4
#define COM_ReadyToProcess     5
#define COM_WAITING_FOR_ACK_ID 6
#define COM_WaitingforSPCL     7

    /* Different USB communication states*/
#define COM_IDLE            1 // Idle state
#define COM_PROCESSING      2 // Communicattion is processing 
#define COM_MSGINQUEUE      3 // Message in the queue 
#define COM_PROC_MSGINQUEUE 4 // Communication is processing while a messagee in the queue

    /* Different CAN-BUS communication states*/
#define CAN_IDLE            1 // Idle state
#define CAN_PROCESSING      2 //Communicattion is processing 
#define CAN_MSGINQUEUE      3 // Message in the queue 
#define CAN_PROC_MSGINQUEUE 4 // Communication is processing while a messagee in the queue

    /*States evolution direction*/
#define BUSIER            0x01 // to a more busy state 
#define FREER             0x00 // to a freer state 

    /* USB and data processing error*/
#define NO_ERROR          0x00
#define COM_TIMEOUT       0x01
#define COM_OVERTRIES     0x02
#define COM_ERRORCRC      0x0A 
#define COM_ERRHEADER     0x0B // Invalid header 
#define COM_ERRCNX        0x0C //
#define COM_ERRUSB        0x0D // Error USB connection??
#define COM_FULLQUEUE     0x0E // Reception when the queue is full 
#define COM_ERRLENGTH     0x0F // The data length is invalid
    
    /*CAN BUS error*/
#define CAN_RCVERR        0xA0 // Error  CAN communication
#define CAN_OVERFLOW      0xB0 // CAN FIFO1 full 
#define CAN_FULLQUEUE     0xC0 // CAN reception queue is full when receiving a new message 
#define CAN_SENDERROR     0xD0 // Error when sending CAN message 
#define CAN_WRONGID       0xE0 // Received message with a wrong Identifier 

    /*Acknowledgement messages*/
#define ACK_SIZE                 2
#define NACK_SIZE                2
#define ERROR_SIZE               1
#define COM_ERRORLENGTH          1 // Error message code 
#define CAN_ERRORLENGTH          1 // Error message code and attached number

    /*Acknowledgement states*/
#define NO_ACK                   0x00
#define ACK                      0x01
#define NACK                     0x02

    /*Acknowledging states*/
#define ACK_WAITING              2
#define ACK_RECEIVED             1
#define ACK_IDLE                 0  

    /*CAN identifier for different mode */
#define IS_CMD_ID(x)  ((x<0xFF)           ? (1):(0))
#define IS_STRM_ID(x) ((x<0x7F && x>0x00) ? (1):(0))
#define IS_RQST_ID(x) ((x<0xFF && x>0x80) ? (1):(0))


/**/

static const uint8_t ERR_MESSAGE[2]  ={
  0xFD};
static const uint8_t NACK_MESSAGE[2] ={
  0xFD, 0x02};
static const uint8_t ACK_MESSAGE[2]  ={
  0xFD, 0x01};

static const uint16_t  CRC_TABLE[256];

/* Exported functions ------------------------------------------------------- */

void COM_ClearUsbBuffer(char* Buffer);
void Bridge_Init(COM_struct* USBtoCAN);
uint16_t crc16(uint16_t crcval, uint8_t newchar);
uint8_t ChangeState_level_Freer(uint8_t State);
uint8_t ChangeState_level_Busier(uint8_t State);
void LED_Init();

/* External variables --------------------------------------------------------*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
