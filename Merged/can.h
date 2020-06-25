#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <driverlib/sysctl.h>
#include <driverlib\fpu.h>
#include <driverlib\gpio.h>
#include <driverlib\pin_map.h>
#include <driverlib\interrupt.h>


typedef struct {
  uint32_t arbid;
  uint8_t data[8];
} CanPacket;

void can_Inject(const CanPacket * packet);

typedef void (*can_LogCallback)(const CanPacket * p);
void can_SetLogging(uint8_t can_id, uint16_t arb_id, uint16_t arb_mask, can_LogCallback cb);
void can_SetFiltering(uint8_t can_id, uint16_t arb_id, uint16_t arb_mask, uint8_t * data, uint8_t * data_mask);
extern void can_ResetFunctions(void);

extern void can_Init(void);
extern void can_setup(void);
void Int_CAN0_Handler_S(void); //Generates interrupt when sending message
void Int_CAN1_Handler_S(void); //Generates interrupt when sending message

void Push_Message(uint32_t _ID, uint8_t data[8]);
void Pop_Message(uint32_t _ID, uint8_t data[8]);

void button_Init(void);
void button_int(void);
void led_Set(bool a, bool b);

void transmit_uart_test(void);

#endif