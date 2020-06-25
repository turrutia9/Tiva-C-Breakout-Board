#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
//#include "list.h"
#include <TM4C1294NCPDT.h>
#include <string.h>
#include <stdbool.h>
#include <inc\hw_gpio.h>
#include <inc\hw_can.h>
#include <inc\hw_ints.h>
#include <inc\hw_nvic.h>
//#include <inc\hw_memmap.h>
#include <inc\hw_sysctl.h>
#include <inc\hw_types.h>
#include <driverlib\can.h>
#include <driverlib\gpio.h>
#include <driverlib\interrupt.h>
#include "driverlib\pin_map.h"
#include "driverlib\sysctl.h"
#include <driverlib\uart.h>

#pragma import(__use_realtime_heap)

#define CAN_TARGET_BAUD 460800
#define RECEIVE_CAN_BASE CAN0_BASE //CAN base messages are being received on
#define SEND_CAN_BASE CAN1_BASE
#define FIFO_MAX_LENGTH 32 //max size of fifo buffer (in bytes)


typedef struct 
{
  uint32_t arbid;
  uint8_t data[8];
} CanPacket;

typedef void (*can_LogCallback)(const CanPacket * p);


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

extern void can_Init(void);


//static void SuperLoopback(void);

void can_Inject(const CanPacket * packet, uint32_t canBase);
tCANMsgObject* SetReceiveAll(uint32_t canbase, size_t fifolen);
void can_SetLogging(uint8_t can_id, uint16_t arb_id, uint16_t arb_mask, can_LogCallback cb);
void can_SetFiltering(uint8_t can_id, uint16_t arb_id, uint16_t arb_mask, uint8_t * data, uint8_t * data_mask);
extern void can_ResetFunctions(void);
extern void can_setup(void);

void Push_Message(uint32_t _ID, uint8_t data[8]);
void Pop_Message(uint32_t _ID, uint8_t data[8]);

void button_Init(void);
void button_int(void);
void led_Set(bool a, bool b);

#endif //CAN_H
