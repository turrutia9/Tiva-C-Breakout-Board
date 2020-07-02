
#include "can.h"
//#include "list.h"
#include <TM4C1294NCPDT.h>
#include <string.h>
#include <stdbool.h>

#include <inc\hw_gpio.h>
#include <inc\hw_can.h>
#include <inc\hw_ints.h>
#include <inc\hw_nvic.h>
//#include <int\hw_memmap.h>
#include <inc\hw_sysctl.h>
#include <inc\hw_types.h>
#include <driverlib\can.h>
#include <driverlib\gpio.h>
#include <driverlib\interrupt.h>
#include "driverlib\pin_map.h"
#include "driverlib\sysctl.h"
#include <driverlib\uart.h>

#pragma import(__use_realtime_heap)

unsigned int messageCount = 0;
unsigned int throttleAdd = 0;
unsigned const int restingThrottle = 0x5F5A;
unsigned int throttleVal = 0;

bool startRecord;
bool startPlayback;

typedef struct LogEntry
{
  uint16_t arb_id;
  uint16_t arb_mask;
  can_LogCallback cb;
} LogEntry;

typedef struct FilterEntry
{
  uint16_t arb_id;
  uint16_t arb_mask;
  uint8_t data[8];
  uint8_t data_mask[8];
} FilterEntry;

static tCANBitClkParms CANBitClk = {
  .ui32SyncPropPhase1Seg = 13,
  .ui32Phase2Seg = 2,
  .ui32SJW = 16,
  .ui32QuantumPrescaler = 15
};

static int can0_txid = 24;
static int can1_txid = 24;

static LogEntry can0_log_entries[3];
static unsigned can0_log_entry_num = 0;
static LogEntry can1_log_entries[3];
static unsigned can1_log_entry_num = 0;
static FilterEntry can0_filter_entries[3];
static unsigned    can0_filter_entry_num = 0;
static FilterEntry can1_filter_entries[3];
static unsigned    can1_filter_entry_num = 0;

void CAN0_Handler(void) //receive on CAN0 and send on CAN1
{
	CANIntClear(CAN0_BASE, CAN_INT_INTID_STATUS); //clear the interrupt flag
	
	States returnedState=updateState(currentCommand); //prime
	while (returnedState != Waiting) //test
	{
		//wait for the next state to start
		returnedState=updateState(currentCommand); //reprime
	}
	
	executeState(returnedState, *currentCommand); //execute the state
	
}

/*
Handler for receiving on CAN1
Does similar thing as Int_CAN0_Handler_R
*/
void CAN1_Handler(void) //receive CAN0 send CAN1
{
    CANIntClear(CAN1_BASE, CAN_INT_INTID_STATUS); //clear the interrupt flag

    tCANMsgObject *inMsg; //object for message coming in
    tCANMsgObject *outMsg; //object for message going out
    uint32_t objID=1; //example used 1; not sure what this means exactly???

    // Configure a receive object.
    inMsg->ui32MsgID=0x400;
    inMsg->ui32MsgIDMask=0x7f8; //message identifier mask
    inMsg->ui32Flags=MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO; //indicate filtering and FIFO are used
    inMsg->ui32MsgLen=8; //for initialization warnings
    *inMsg->pui8MsgData=0x9; //for initialization warnings

    //CAN base, object number to read (1-32), message object fields, clear associated interrupt
    CANMessageGet(CAN1_BASE, objID, inMsg, true); //extract the message

    //configure transmit message with the data from the message being taken in
    outMsg->ui32MsgID = inMsg->ui32MsgID; //set the ID
    outMsg->ui32Flags = inMsg->ui32Flags; //copy the flags
    outMsg->ui32MsgLen = inMsg->ui32MsgLen; //length is 8 bytes
    outMsg->pui8MsgData = inMsg->pui8MsgData; //message data

    CANMessageSet(CAN0_BASE, objID, outMsg, MSG_OBJ_TYPE_TX); //Send the message
}


void can_Inject(const CanPacket * packet)
{
  tCANMsgObject sMsgObjectTx;
  //
  // Configure and start transmit of message object.
  //
  sMsgObjectTx.ui32MsgID = packet->arbid;
  sMsgObjectTx.ui32Flags = 0;
  sMsgObjectTx.ui32MsgLen = 8;
  sMsgObjectTx.pui8MsgData = (uint8_t *)packet->data;
  CANMessageSet(CAN1_BASE, 32, &sMsgObjectTx, MSG_OBJ_TYPE_TX);
}

void SetReceiveAll(uint32_t canbase, size_t fifolen, void (*interrupt)(void))
{
  CANInit(canbase);
  CANBitTimingSet(canbase, &CANBitClk);
  CANEnable(canbase);
  
  tCANMsgObject sMsgObjectRx;
  // Configure a receive object.
  sMsgObjectRx.ui32MsgID = 0x000;
  sMsgObjectRx.ui32MsgIDMask = 0x000;
  sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO | MSG_OBJ_RX_INT_ENABLE;
  sMsgObjectRx.ui32MsgLen = 8;
  sMsgObjectRx.pui8MsgData = 0;
  //
  // The first fifolen - 1 message objects have the MSG_OBJ_FIFO set to indicate
  // that they are part of a FIFO.
  //
  for(int i = 1 ; i < fifolen ; i ++) 
    CANMessageSet(canbase, i, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
  //
  // Last message object does not have the MSG_OBJ_FIFO set to indicate that
  // this is the last message.
  //
  sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_RX_INT_ENABLE;
  CANMessageSet(canbase, fifolen, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
  
  CANIntRegister(canbase, interrupt);
  CANIntEnable(canbase, CAN_INT_MASTER);
}

void can_SetLogging(uint8_t can_id, uint16_t arb_id, uint16_t arb_mask, can_LogCallback cb)
{
  if(can_id == 0)
  {
    const unsigned max_entries = sizeof(can0_log_entries)/sizeof(*can0_log_entries);
    unsigned idx = can0_log_entry_num;
    if(idx < max_entries)
    {
      can0_log_entries[idx].arb_id = arb_id;
      can0_log_entries[idx].arb_mask = arb_mask;
      can0_log_entries[idx].cb = cb;
      can0_log_entry_num ++;
    }
  }
  else if(can_id == 1)
  {
    const unsigned max_entries = sizeof(can1_log_entries)/sizeof(*can1_log_entries);
    unsigned idx = can1_log_entry_num;
    if(idx < max_entries)
    {
      can1_log_entries[idx].arb_id = arb_id;
      can1_log_entries[idx].arb_mask = arb_mask;
      can1_log_entries[idx].cb = cb;
      can1_log_entry_num ++;
    }
  }
}

void can_SetFiltering(uint8_t can_id, uint16_t arb_id, uint16_t arb_mask, uint8_t * data, uint8_t * data_mask)
{
  if(can_id == 0)
  {
    const unsigned max_entries = sizeof(can0_filter_entries)/sizeof(*can0_filter_entries);
    unsigned idx = can0_filter_entry_num;
    if(idx < max_entries)
    {
      can0_filter_entries[idx].arb_id = arb_id;
      can0_filter_entries[idx].arb_mask = arb_mask;
      memcpy(can0_filter_entries[idx].data, data, 8);
      memcpy(can0_filter_entries[idx].data_mask, data_mask, 8);
      can0_filter_entry_num ++;
    }
  }
  else if(can_id == 1)
  {
    const unsigned max_entries = sizeof(can1_filter_entries)/sizeof(*can1_filter_entries);
    unsigned idx = can1_filter_entry_num;
    if(idx < max_entries)
    {
      can1_filter_entries[idx].arb_id = arb_id;
      can1_filter_entries[idx].arb_mask = arb_mask;
      memcpy(can1_filter_entries[idx].data, data, 8);
      memcpy(can1_filter_entries[idx].data_mask, data_mask, 8);
      can1_filter_entry_num ++;
    }
  }
}

void can_ResetFunctions()
{
  can0_log_entry_num = 0;
  can0_filter_entry_num = 0;
  can1_log_entry_num = 0;
  can1_filter_entry_num = 0;
}

void button_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	
	GPIOPinTypeGPIOInput(GPIOJ_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPadConfigSet(GPIOJ_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntRegister(GPIOJ_AHB_BASE, button_int);
	GPIOIntTypeSet(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
	startRecord = false;
	startPlayback = false;
}

//wut -- sets the gpio settings 5head
void button_int(void)
{
	uint8_t data = GPIOPinRead(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 |  GPIO_INT_PIN_1);
	
	if((data & 0x2) == 0)
	{
		if((startRecord == false) && (startPlayback == false))
		{
			startRecord = true;
			led_Set(1,0);
		}
		else if((startRecord == true) && (startPlayback == false))
		{
			startRecord = false;
			led_Set(0,0);
		}
	}
	else if((data & 0x1) == 0)
	{
		if((startPlayback == false) && (startRecord == false))
		{
			startPlayback = true;
			led_Set(0,1);
		}
		else if((startPlayback == true) && (startRecord == false))
		{
			startPlayback = false;
			led_Set(0,0);
		}
	}
	for(int i = 0; i < 1000000; )
	{
		++i;
	}
	GPIOIntClear(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
}

void can_setup(void){
	//set incoming traffic on can0 and can1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)){
	}
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)){
	}
	
	CANInit(CAN0_BASE);
	CANInit(CAN1_BASE);
	
	//CANSetBitTiming(CAN1_BASE, &CANBitClk);
	IntMasterEnable();
	SetReceiveAll(CAN0_BASE, 8, CAN0_Handler);
  SetReceiveAll(CAN1_BASE, 8, CAN1_Handler);
	
	can_ResetFunctions();
}

void can_Init(void)
{
  SYSCTL->RCGCGPIO |= 0x03; // GPIOA and GPIOB
  SYSCTL->RCGCCAN |= 0x03; // CAN1 and CAN0
  
  // Incoming can on GPIOA, CAN0
  // Pin 0,1 digital enable
  GPIOA_AHB->DEN |= 0x03; // B0000.0011
  // Pin 0 input, pin 1 output
  GPIOA_AHB->DIR |= 0x02; // B0000.0010
  // Pin 0,1 low
  GPIOA_AHB->AFSEL |= 0x03; // B0000.0011
  // Clear and set port mux for GPIOA 0,1
  GPIOA_AHB->PCTL &= ~0x000000FF;
  GPIOA_AHB->PCTL |= 0x00000077;
  
  // Incoming can on GPIOB, CAN1
  // Pin 0,1 digital enable
  GPIOB_AHB->DEN |= 0x03; // B0000.0011
  // Pin 0 input, pin 1 output
  GPIOB_AHB->DIR |= 0x02; // B0000.0010
  // Pin 0,1 low
  GPIOB_AHB->AFSEL |= 0x03; // B0000.0011
  // Clear and set port mux for GPIOB 4,5
  GPIOB_AHB->PCTL &= ~0x000000FF;
  GPIOB_AHB->PCTL |= 0x00000077;
  
  //IntEnable(INT_CAN0);
  //IntEnable(INT_CAN1);
  IntMasterEnable();
  SetReceiveAll(CAN0_BASE, 8, CAN0_Handler);
  SetReceiveAll(CAN1_BASE, 8, CAN1_Handler);
  
  can_ResetFunctions();
	
	
  //SuperLoopback();
  
  /*
  CanPacket p = {
    .arbid = 0x130,
    .data = { 0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE, 0xC0, 0xFF },
  };
  
  can_Inject(&p);
  */
  
  //can_ReadBlock(&p);
}
