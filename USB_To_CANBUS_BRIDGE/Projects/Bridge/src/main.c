/**
  ******************************************************************************
  * @file    main.c
  * @author  Rami HADJ TAIEB
  * @version V1.0.0
  * @date    07-March-2014
  * @brief   Rov USB to CAN bridge main file
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "BridgeCAN_Lib.h"
#include "BridgeCOM_Lib.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*-------------------exemple for crc testing -------------------*/
/*to test the message decompression we should match the same 
values on the crc and crc_pc attributes of the  bridge structure*/
/*Example of CMD mode Message: 
{0xFC ,0x05, 0x08,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF,0x02,0x03,0xB5,0xDB}
  Example of CMD mode Message: 
{0xFD ,0x05, 0x08,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF,0x02,0x03,0xB5,0xDB}
  Example of STRM mode Message: 
{0xFE ,0x05, 0x08,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF,0x02,0x03,0xB5,0xDB}
*/
uint8_t crc_test[13]={ 
  0xFC ,0x05, 0x08,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF,0x02,0x03,0xB5,0xDB
};
uint32_t Test_Counter=1;
uint16_t crcvalue_test=0xFFFF;

/* Extern variables ----------------------------------------------------------*/
  /************ USB CDC variables***********/
  extern __IO  uint8_t Receive_Buffer[16];
  extern __IO  uint32_t Receive_length ;
  extern __IO  uint32_t length ;
  uint8_t Send_Buffer[16];
  uint32_t packet_sent=1;
  uint32_t packet_receive=1;
  extern COM_struct Bridge;

  
  /*********CAN variables********/
  uint32_t CANpacket_sent;
  uint32_t CANpacket_receive; 
  uint8_t k=0;
  
/* Private function prototypes -----------------------------------------------*/ 
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main.
* Descriptioan    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
  int main(void)
{ /*system configuration*/
  
  LED_Init();
  
  STM_EVAL_LEDToggle(LED3);
  STM_EVAL_LEDToggle(LED4);
  STM_EVAL_LEDToggle(LED5);
  STM_EVAL_LEDToggle(LED6);
  STM_EVAL_LEDToggle(LED7);
  STM_EVAL_LEDToggle(LED8);
  STM_EVAL_LEDToggle(LED9);
  STM_EVAL_LEDToggle(LED10);
  
  Set_System();
  /*CAN BUS configuration*/
  CAN_Config();
  /*USB CDC configuration*/
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED6);
  STM_EVAL_LEDOff(LED7);
  STM_EVAL_LEDOff(LED8);
  STM_EVAL_LEDOff(LED9);
  STM_EVAL_LEDOff(LED10);
  
  /*Systick configration 100µs*/
  //NVIC_SetPriority(SysTick_IRQn,1);
  if (SysTick_Config(SystemCoreClock /10000))
  { 
    /* Capture error */ 
    while(1);
  }
  
  /*Varible initialization*/
  Bridge_Init(&Bridge);
  //uint8_t k=0;
  CANpacket_receive=0;
  CANpacket_sent=0;
    
  while (1)
  { //test seection 
    /*k++;
    Bridge.TxMessage1.Data[0]=k;
    CAN_Transmit(CAN1,&Bridge.TxMessage1);
    for (k=0;k<11;k++){
      CDC_Receive_DATA(crc_test[k]);
      crcvalue_test=crc16(crcvalue_test,crc_test[k]); } //for crc computation*/
   CAN_Transmit(CAN1,&Bridge.TxMessage1);
    while(bDeviceState==CONFIGURED) 
      { 
        STM_EVAL_LEDToggle(LED7);/* 128 synchronization without getting an acknowledgement*/
       if(Bridge.COM_Acknowledgement.Nb_Check >0x80)
        {
          Bridge.COM_Err.ErrorId=COM_TIMEOUT;
          Bridge.COM_Acknowledgement.Nb_Check=0;
          Bridge.CAN_Acknowledgement.State=ACK_IDLE;
        }
        
        else if(Bridge.COM_Acknowledgement.Nb_Tries >0x80)/* 128 times with non valid acknowledgement  received*/
              {
                Bridge.COM_Err.ErrorId=COM_OVERTRIES;
                Bridge.CAN_Acknowledgement.State=ACK_IDLE;
                Bridge.COM_Acknowledgement.Nb_Tries=0;
                STM_EVAL_LEDToggle(LED6);
              }
        }
  }
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number*/
      printf("Wrong parameters value: file %s on line %d\r\n", file, line) 

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
