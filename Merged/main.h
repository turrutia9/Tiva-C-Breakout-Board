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

int binarySearch(int *list, int listLength, int item); //binary search to find message of interest
CommandPacket parseUARTMsg(uint8_t *rawCmd);
States updateState(CommandPacket *cmd);
void executeState(States state, CommandPacket data);
uint16_t getNoise();