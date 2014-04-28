/**
  ******************************************************************************
  * @file    stm32_it.c
  * @author  Hadj Taïeb Rami
  * @version V1.0.0
  * @date    07-March-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "usb_istr.h"
#include "BridgeCAN_Lib.h"
#include "BridgeCOM_Lib.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* External variables ---------------------------------------------------------*/    
 extern   CanRxMsg RxMessage1;
 extern   COM_struct Bridge;
 extern   uint32_t CANpacket_sent;
 extern   uint32_t CANpacket_receive; 
 uint16_t CANTX_Counter=0;
 uint8_t CAN_mbox;

/******************************************************************************/
/*                 STM32F30x Peripherals Interrupt Handlers                   */
/******************************************************************************/
/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                          */
/******************************************************************************/
/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{ if(bDeviceState==CONFIGURED){  

      /********************************************************************************
      ********************* Message received from CAN BUS (ROV) ***********************
      *********************************************************************************/  
      if(CANpacket_receive==1 && Bridge.CAN_State != CAN_MSGINQUEUE && Bridge.CAN_State != CAN_PROC_MSGINQUEUE)
      { 
        if(CAN_SaveMsg(&Bridge.RxMessage1,&Bridge.COM_Lastmessage.CDC_ToPC_Buffer))
        {
          Bridge.CAN_State=ChangeState_level_Busier(Bridge.CAN_State);
          STM_EVAL_LEDToggle(LED9);
        }
        else 
        {
          Bridge.CAN_State=ChangeState_level_Freer(Bridge.CAN_State);
          STM_EVAL_LEDToggle(LED10);
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
        if( Bridge.COM_Status==COM_ReadyToProcess)
        { 
        
          if(Bridge.COM_State == COM_PROCESSING)
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
              while(((Bridge.CAN_Err.ErrorData=CAN_TransmitStatus(CAN1, CAN_mbox))  !=  CAN_TxStatus_Ok) && (CANTX_Counter !=  0xFFFF))
              {
                CANTX_Counter++;
              }
              if(CANTX_Counter!=0xFFFF)
              {
                if (Bridge.COM_State == COM_PROC_MSGINQUEUE)
                    COM_SaveMsg(&Bridge.Reception_Buffer,&Bridge.COM_Lastmessage.CDC_FromPC_Buffer);
                  Bridge.COM_State=ChangeState_level_Freer(Bridge.COM_State);
              }
               else
                  Bridge.CAN_Err.ErrorId=CAN_SENDERROR;
            }               
      }    
      
     
  
      /********************************************************************************
      ********************* Error Message on the USB port (pc) ***********************
      *********************************************************************************/  
    if (Bridge.COM_Err.ErrorId)
    {     
      STM_EVAL_LEDToggle(LED5);
      //COM_SendSpecial(Bridge.COM_Err.ErrorId); to be replaced by a table to send
      Bridge.COM_Err.ErrorId=0;    
    }
    
      /********************************************************************************
      ********************* Error Message (ROV) ***********************
      *********************************************************************************/
    if (Bridge.CAN_Err.ErrorId)
    { 
      STM_EVAL_LEDToggle(LED8);
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
         COM_SendMsg(&Bridge.COM_Lastmessage.CDC_ToPC_Buffer);
         Bridge.COM_Acknowledgement.State=ACK_WAITING;
       }
    }   
     else if(Bridge.COM_Acknowledgement.State==ACK_WAITING)
     {
       Bridge.COM_Acknowledgement.Nb_Check++;
       STM_EVAL_LEDToggle(LED6);
       COM_SendMsg(&Bridge.COM_Lastmessage.CDC_ToPC_Buffer);
     }
  }
  STM_EVAL_LEDOff(LED7); 
}

/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)

{
  USB_Istr();
}

/*******************************************************************************
* Function Name  : CAN1_RX1_IRQHandler
* Description    : This function handles CAN1 RX1 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN1_RX1_IRQHandler(void)
{ if(CAN_GetITStatus(CAN1,CAN_IT_FMP1)== SET)
  {
   // if(Bridge.CAN_State!= CAN_PROC_MSGINQUEUE && Bridge.CAN_State!= CAN_PROCESSING )
   //{
      CAN_Receive(CAN1, CAN_FIFO1, &Bridge.RxMessage1);
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
        if(CAN_GetITStatus(CAN1,CAN_FLAG_FF1)== SET)
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

/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
void USB_FS_WKUP_IRQHandler(void)
#else
void USBWakeUp_IRQHandler(void)
#endif
{
  EXTI_ClearITPendingBit(EXTI_Line18);
}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : PPP_IRQHandler
* Description    : This function handles PPP interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

