

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* HOW IT WORKS
DATA DIRECTION [ PC --> ROV ]

PC 
1/ C# send data byte via serial 

F3
1/  Read serial data byte  (Read data and CRC check it)
2/  send data byte via CAN (max 8 bytes each time) 

F4
1/ CAN Rx Interrupt
2/ Process down packet (put into var for further use)


DATA DIRECTION [ ROV --> PC ]

F4
1/ Process up packet
2/ Send via CAN

F3
1/ CAN Rx interrupt  
2/ Send data byte via USB (CDC) (max 64 bytes each time)

PC
1/C# UART Rx interrupt (Read data and CRC check it)
2/ Process up data packet (put into var for use by GUI) 
*/




