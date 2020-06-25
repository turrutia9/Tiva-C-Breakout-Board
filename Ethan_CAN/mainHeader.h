#ifndef _MAINHEADER_H
#define _MAINHEADER_H

#define PART_TM4C1294NCPDT

#include <TM4C1294NCPDT.h>
#include <stdbool.h>
#include <string.h>
#include <driverlib\sysctl.h>
#include <driverlib\fpu.h>
#include <inc\hw_uart.h>
#include <driverlib\uart.h>
#include <driverlib\gpio.h>
#include <driverlib\pin_map.h>
#include <driverlib\interrupt.h>
#include <inc\hw_ints.h>
#include <driverlib\systick.h>
#include "can.h"
#include "list.h"

#define PERIOD 0.5 //period for systick clock


typedef enum {Injection, PassThrough, Substitute, Drop, DropOnce, Clear, Disconnected, Waiting} States; //different states for uC

/*
Struct for the various parts of the uart command packet
Each field will be set in the parse function
this struct is then passed around to various functions
*/
typedef struct
{
	int UARTMsg;
	bool noise;
	uint32_t immediate;
	uint8_t dataLength;
} CommandPacket;

CommandPacket *currentCommand; //global uart command packet; used for CAN interrupt handlers
tCANMsgObject *inCanMsg; //global CAN object for the incoming CAN messages



volatile uint8_t fifoLength=0; //global variable to store the fifo length of can

//#pragma (import __use_no_semihosting)

//MY DECLARATIONS
void initCANModules(tCANBitClkParms *CANBitClk);
//void Int_CAN0_Handler_R(void); //Generates interrupt when receiving message
void Int_CAN0_Handler_S(void); //Generates interrupt when sending message
void Int_CAN1_Handler_S(void); //Generates interrupt when sending message
//void Int_CAN1_Handler_R(void); //Generates interrupt when receiving message
int binarySearch(int *list, int listLength, int item); //binary search to find message of interest
CommandPacket parseUARTMsg(uint8_t *rawCmd);
States updateState(CommandPacket *cmd);
void executeState(States state, CommandPacket data);
uint16_t getNoise();


//GIVEN DECLARATIONS
void int_Init(void);
void led_Init(void);
void led_Set(bool a, bool b);
void led_Byte(uint8_t byte);
void uart_init(void);
void pcsr_Init(void);
void pcsr_ReadData(uint8_t * buf, size_t size);
void pcsr_WriteData(const uint8_t * buf, size_t size);
void LogHandler(const CanPacket * packet);
void HardFault_Handler(void);


#endif // _MAIN_HEADER_H
