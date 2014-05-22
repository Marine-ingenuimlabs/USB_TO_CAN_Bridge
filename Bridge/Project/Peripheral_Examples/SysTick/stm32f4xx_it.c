/**
  ******************************************************************************
  * @file    SysTick/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup SysTick_Example
  * @{
  */ 
/*
 * The USB data must be 4 byte aligned if DMA is enabled. This macro handles
 * the alignment, if necessary (it's actually magic, but don't tell anyone).
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
    extern volatile uint32_t ticker, downTicker;
    extern USB_OTG_CORE_HANDLE  USB_OTG_dev ;
    extern CanRxMsg RxMessage1;
    extern COM_struct Bridge;
    extern uint32_t CANpacket_sent;
    extern uint32_t CANpacket_receive; 
    uint32_t CANTX_Counter=0;
    uint8_t CAN_mbox;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{  

      /********************************************************************************
      ********************* Message received from CAN BUS (ROV) ***********************
      *********************************************************************************/
      if(CANpacket_receive==1 && Bridge.CAN_State != CAN_MSGINQUEUE && Bridge.CAN_State != CAN_PROC_MSGINQUEUE)
      { 
        if(CAN_SaveMsg(&Bridge.RxMessage1,&Bridge.COM_Lastmessage.CDC_ToPC_Buffer))
        {
          Bridge.CAN_State=ChangeState_level_Busier(Bridge.CAN_State);
          //STM_EVAL_LEDToggle(LED9);
        }
        else 
        {
          Bridge.CAN_State=ChangeState_level_Freer(Bridge.CAN_State);
          //STM_EVAL_LEDToggle(LED10);
        }
         /*Send CAN message if queue isn't empty */      
        if (Bridge.CAN_State == CAN_MSGINQUEUE || Bridge.CAN_State == CAN_PROC_MSGINQUEUE)
        {
           COM_SendMsg(&Bridge.COM_Lastmessage.CDC_ToPC_Buffer);
           Bridge.COM_Acknowledgement.State=ACK_WAITING;
           Bridge.COM_Acknowledgement.COM_ACK=NO_ACK;
        }
       CANpacket_receive=0; 
      }
      
      /********************************************************************************
      ********************* Message received from USB port (pc) ***********************
      *********************************************************************************/
       
        
          if(Bridge.COM_State == COM_PROCESSING &&   Bridge.COM_Status==COM_ReadyToProcess)
          {
            COM_SaveMsg(&Bridge.Reception_Buffer,&Bridge.COM_Lastmessage.CDC_FromPC_Buffer);
            Bridge.COM_State=ChangeState_level_Busier(Bridge.COM_State);
            Bridge.COM_Status=COM_WaitingforHeader;
          }
          /*Send serial message if queue isn't empty */
          if ((Bridge.COM_State == COM_MSGINQUEUE || Bridge.COM_State == COM_PROC_MSGINQUEUE)  )
            {  CANTX_Counter=0;
              // make the function to send CAN messages through serial          
              CAN_SendMsg(&Bridge.COM_Lastmessage.CDC_FromPC_Buffer);
              CAN_mbox=CAN_Transmit(CAN1,&Bridge.TxMessage1);
              while(((Bridge.CAN_Err.ErrorData=CAN_TransmitStatus(CAN1, CAN_mbox))  !=  CAN_TxStatus_Ok) && (CANTX_Counter !=  0x1FFFFFF))
              {
                CANTX_Counter++;
                GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
              }
              
              if(CANTX_Counter < 0x1FFFFFF)
              { GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
                Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State);
                if (Bridge.COM_State == COM_PROCESSING &&   Bridge.COM_Status==COM_ReadyToProcess)
                { 
                  
                    COM_SaveMsg(&Bridge.Reception_Buffer,&Bridge.COM_Lastmessage.CDC_FromPC_Buffer);
                    Bridge.COM_State=ChangeState_level_Busier(Bridge.COM_State);
                    Bridge.COM_Status=COM_WaitingforHeader; 
                }
                  
              }
              else{
                  Bridge.CAN_Err.ErrorId=CAN_SENDERROR;
                  GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
              }
                         
      }

     

      /********************************************************************************
      ********************* Error Message on the USB port (pc) ***********************
      *********************************************************************************/  
    if (Bridge.COM_Err.ErrorId)
    {     
      //STM_EVAL_LEDToggle(LED5);
      //COM_SendSpecial(Bridge.COM_Err.ErrorId); to be replaced by a table to send
      Bridge.COM_Err.ErrorId=0;    
    }

      /********************************************************************************
      ********************* Error Message (ROV) ***********************
      *********************************************************************************/
    if (Bridge.CAN_Err.ErrorId)
    { 
      //STM_EVAL_LEDToggle(LED8);
      //COM_SendSpecial(Bridge.CAN_Err.ErrorId);
      Bridge.CAN_Err.ErrorId=0;
    }

      /********************************************************************************
      ********************* Acknowledgement Message (PC) ***********************
      *********************************************************************************/
     if (Bridge.COM_Acknowledgement.State==ACK_RECEIVED)
    {  
      if(Bridge.COM_Acknowledgement.COM_ACK==ACK)
      {
         Bridge.COM_Acknowledgement.State=ACK_IDLE;
         Bridge.CAN_State=ChangeState_level_Freer(Bridge.CAN_State);
      }
       else
       {
         Bridge.COM_Acknowledgement.Nb_Tries++;
         if (Bridge.COM_Acknowledgement.Nb_Tries==4)
         {
           Bridge.COM_Acknowledgement.State=ACK_IDLE;
           Bridge.COM_Acknowledgement.Nb_Tries=0;
         }
         if((Bridge.COM_Acknowledgement.Nb_Tries%2)==0 ){
           COM_SendMsg(&Bridge.COM_Lastmessage.CDC_ToPC_Buffer);
           Bridge.COM_Acknowledgement.State=ACK_WAITING;
         }
       }
    }   
     else if(Bridge.COM_Acknowledgement.State==ACK_WAITING)
     {
       Bridge.COM_Acknowledgement.Nb_Check++;
       if(Bridge.COM_Acknowledgement.Nb_Check==4)
       {
         Bridge.CAN_State=ChangeState_level_Freer(Bridge.CAN_State);
         Bridge.COM_Acknowledgement.State=ACK_IDLE;
         Bridge.COM_Acknowledgement.Nb_Check=0;
         
       }
       /*if((Bridge.COM_Acknowledgement.Nb_Check %2) == 0){
       COM_SendMsg(&Bridge.COM_Lastmessage.CDC_ToPC_Buffer);
       }*/
     }
  
  //STM_EVAL_LEDOff(LED7);
}


/*******************************************************************************
* Function Name  : CAN1_RX1_IRQHandler
* Description    : This function handles CAN1 RX1 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{ if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)== SET)
  {
   // if(Bridge.CAN_State!= CAN_PROC_MSGINQUEUE && Bridge.CAN_State!= CAN_PROCESSING )
   //{
      GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
      CAN_Receive(CAN1, CAN_FIFO0, &Bridge.RxMessage1);
      CANpacket_receive=1; 
      Bridge.CAN_State=ChangeState_level_Busier(Bridge.CAN_State);
      //CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1) //Don't need this line the intterruption is cleared automatically when read
   /* }
    else 
    {
      Bridge.CAN_Err.ErrorId=CAN_FULLQUEUE;
      Bridge.CAN_Err.ErrorData= 0;
    }*/
  }
   else
   {    /*This case has to be treated as an important problem of using the multiport communication: The priority of the CAN communication have to be modified after tests or make an automated algorithm that (numerically ) */
        if(CAN_GetITStatus(CAN1,CAN_FLAG_FF0)== SET)
        {
          //CAN_ClearITPendingBit( CAN1,CAN_FLAG_FF1);
          Bridge.CAN_Err.ErrorId=CAN_OVERFLOW;
          Bridge.CAN_Err.ErrorData=CAN_MessagePending( CAN1,CAN_FIFO1);
        }
        /*Interrupt when error iss reported by the CAN BUS controller*/
        else if(CAN_GetITStatus(CAN1,CAN_IT_ERR)== SET)
            {
              Bridge.CAN_Err.ErrorId=CAN_RCVERR;
              Bridge.CAN_Err.ErrorData=CAN_GetLastErrorCode(CAN1);            
            }
    }
}


void NMI_Handler(void)       {}
void HardFault_Handler(void) { ColorfulRingOfDeath(); }
void MemManage_Handler(void) { ColorfulRingOfDeath(); }
void BusFault_Handler(void)  { ColorfulRingOfDeath(); }
void UsageFault_Handler(void){ ColorfulRingOfDeath(); }
void SVC_Handler(void)       {}
void DebugMon_Handler(void)  {}
void PendSV_Handler(void)    {}

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}




/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
