/******************** (C) COPYRIGHT 2013 HARROV V1.0 ********************
* File Name          : Main.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 25/04/2014
* Description        : Main program of the USB to CAN bridge 
****************************************************************************/
/*Test frame : 0xFC ,0x05, 0x08,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF,0x02,0x03,0xB5,0xDB*/
/* this code needs standard functions used by STM32F4xx.
 */
 
 /* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/*
 * The USB data must be 4 byte aligned if DMA is enabled. This macro handles
 * the alignment, if necessary.
 */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
/* Private variables ---------------------------------------------------------*/
volatile uint32_t ticker, downTicker;
extern COM_struct Bridge;
uint32_t CANpacket_receive;
uint32_t CANpacket_sent;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void LED_Config();
void ColorfulRingOfDeath(void);
/*******************************************************************************
* Function Name  : main
* Description    : Checking for timeout and over tries 
* Input          : None
* Output         : None.
* Return         : None.
*******************************************************************************/
void main(void)
{
	/* Set up the system clocks */
	SystemInit();
        /* Set up the CAN 1 */
        CAN_Config();
        /* Setup USB */
	USBD_Init(&USB_OTG_dev,
	            USB_OTG_FS_CORE_ID,
	            &USR_desc,
	            &USBD_CDC_cb,
	            &USR_cb);
        
        /* Setup SysTick or CROD! */
	if (SysTick_Config(SystemCoreClock /20000))
	{
		ColorfulRingOfDeath();
	}
        
        /*Variables Initialization*/
        Bridge_Init(&Bridge);
        CANpacket_receive=0;
        CANpacket_sent=0;

	/* LED initialisation */
	LED_Config();
        
        GPIO_SetBits(GPIOD, GPIO_Pin_12);//led3 orange
        GPIO_SetBits(GPIOD, GPIO_Pin_13);//led4 
        GPIO_SetBits(GPIOD, GPIO_Pin_14);//led 5 rouge
        GPIO_SetBits(GPIOD, GPIO_Pin_15);//led 6 blue
        
	while (1)
	{
		
	}

}

void LED_Config()
{
	/* STM32F4 discovery LEDs */
	GPIO_InitTypeDef LED_Config;

	/* Always remember to turn on the peripheral clock...  If not, you may be up till 3am debugging... */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	LED_Config.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	LED_Config.GPIO_Mode = GPIO_Mode_OUT;
	LED_Config.GPIO_OType = GPIO_OType_PP;
	LED_Config.GPIO_Speed = GPIO_Speed_25MHz;
	LED_Config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &LED_Config);
}

/*
 * Call this to indicate a failure.  Blinks the STM32F4 discovery LEDs
 * in sequence.  At 168Mhz, the blinking will be very fast - about 5 Hz.
 * Keep that in mind when debugging, knowing the clock speed might help
 * with debugging.
 */
void ColorfulRingOfDeath(void)
{
	uint16_t ring = 1;
	while (1)
	{
		uint32_t count = 0;
		while (count++ < 500000);

		GPIOD->BSRRH = (ring << 12);
		ring = ring << 1;
		if (ring >= 1<<4)
		{
			ring = 1;
		}
		GPIOD->BSRRL = (ring << 12);
	}
}