
#include <TM4C1294NCPDT.h> //structs defined here
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include <driverlib/sysctl.h>
#include <driverlib\fpu.h>
#include <driverlib\gpio.h>
#include <driverlib\pin_map.h>
#include <driverlib\interrupt.h>
#include "can.h"
#include "list.h"

//uart includes
#include <inc\hw_uart.h>
#include <driverlib\uart.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <inc\hw_ints.h>
#include <inc/hw_memmap.h>

//#pragma (import __use_no_semihosting)

uint32_t g_ui32SysClock;

void int_Init()
{
	IntPrioritySet(INT_GPIOJ_TM4C129,0x80);
	IntPrioritySet(INT_CAN0_TM4C129,0x00);
	IntPrioritySet(INT_CAN1_TM4C129,0x00);
}

void led_Init()
{
  SYSCTL->RCGCGPIO |= 0x20; // GPIOF
  
  // Pins [4:0] digital enable
  GPIOF_AHB->DEN |= 0x11;
  // Pins [4:0] output
  GPIOF_AHB->DIR |= 0x11;
}
void led_Set(bool a, bool b)
{
  uint32_t dval = 0;
  if(a) dval |= 0x01;
  if(b) dval |= 0x10;
  GPIOF_AHB->DATA &= ~(0x11);
  GPIOF_AHB->DATA |= (dval & 0x11);
}
void led_Byte(uint8_t byte)
{
  for(int b = 0 ; b < 8 ; b ++)
  {
    led_Set(false, false);
    for(unsigned i = 0 ; i < 800000 ; i ++);
    if(byte & (1 << b))
      led_Set(false, true);
    else
      led_Set(true, false);
    for(unsigned i = 0 ; i < 400000 ; i ++);
  }
  led_Set(false, false);
}

void uart_init(){
	g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
	uint64_t baud = 460800; //might have to change this to 38400
	UARTDisable(UART2_BASE); //disable the uart
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2); //enable the peripheral
	
	//write the brd and clock values, sets the baud of the peripheral
	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), baud,
										  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
											UART_CONFIG_PAR_NONE));
	UARTEnable(UART2_BASE);
	//wait to get data from the computer and send it back
	//make this interrupt driven (handler for rx)
}
void get_interrupt(){
	uint32_t status;
	status = ROM_UARTIntStatus(UART0_BASE, true); //get interrupt status
	ROM_UARTIntClear(UART0_BASE, status); //clear asserted interrupts
	//while there are characters to receive in the FIFO buffer
	while (ROM_UARTCharsAvail(UART0_BASE)){
		//read next char from uart and write it back
		ROM_UARTCharPutNonBlocking(UART0_BASE, ROM_UARTCharGetNonBlocking(UART0_BASE));
		//blink to show transfer is occuring
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
		SysCtlDelay(g_ui32SysClock / (1000 * 3)); //1 millisecond
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0); //turn off led when trasferred
	}
}
void uart_send(const uint8_t* buffer, uint32_t count){
	while(count--){
		//put nonblocking characters
		ROM_UARTCharPutNonBlocking(UART0_BASE, *buffer++);
	}
}
void uart_test(){
	g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	ROM_IntMasterEnable();
	
	GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	ROM_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
	
	ROM_IntEnable(INT_UART0);
  ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	uart_send((uint8_t *)"\033[2JEnter text: ", 16);
	
	while (1){}
}

//reference from this code
void pcsr_Init()
{
  SYSCTL->RCGCGPIO |= 0x8;
  SYSCTL->RCGCUART |= 0x4;
  
  // Pins [4:5] digital enable
  GPIOD_AHB->DEN |= 0x30;
  // Pin 5 output
  GPIOD_AHB->DIR |= 0x20;
  // Pin 5 high
  //GPIOA->DATA |= 0x20;
  // Pins [4:5] alternate functionality (UART2 Rx/Tx)
  GPIOD_AHB->AFSEL |= 0x30;
  // Clear and set port mux for GPIOD pins [4:5]
  GPIOD_AHB->PCTL &= ~0xFF0000;
  GPIOD_AHB->PCTL |= 0x110000;
  
	//target baud of the can bus
  uint64_t baud = 460800;
  
  // Disable UART2
  UART2->CTL &= ~0x1;
  // Clear and set integer baud-rate divisor
  UART2->IBRD &= ~0xFFFF;
  UART2->IBRD |= (16000000lu / (16lu * baud)) & 0xFFFF;
  // Clear and set fractional baud-rate divisor
  UART2->FBRD &= ~0x3F;
  UART2->FBRD |= (((16000000lu * 128lu) / (16lu * baud) + 1)/2) & 0x3F;
  // Clear and set line control (8-bit, no FIFOs, 1 stop bit)
  UART2->LCRH &= ~0xFF;
  UART2->LCRH |= 0x70;
  // Clear and set clock source (PIOSC)
  UART2->CC &= ~0xF;
  UART2->CC |= 0x05;
  // Enable DMA on Tx line
  //UART2->DMACTL |= 0x2;
  // Enable interrupts for data received
  //UART2->IM |= 0x10;
  // Enable loopback operation
  //UART2->CTL |= 0x80;
  // Enable UART2
  UART2->CTL |= 0x1;
}

void pcsr_ReadData(uint8_t * buf, size_t size)
{
  for(unsigned i = 0 ; i < size ; i ++)
    buf[i] = UARTCharGet(UART2_BASE);
}
void pcsr_WriteData(const uint8_t * buf, size_t size)
{
  for(unsigned i = 0 ; i < size ; i ++)
    UARTCharPut(UART2_BASE, buf[i]);
}

#define FUNCTION_LOG    0x00
#define FUNCTION_FILTER 0x01
#define FUNCTION_RESET  0x02

void LogHandler(const CanPacket * packet)
{
  uint8_t buf[14];
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = packet->arbid & 0xFF;
  buf[5] = packet->arbid >> 8;
  
  for(unsigned i = 0 ; i < 8 ; i++)
  {
    buf[i + 6] = packet->data[i];
  }
  
  pcsr_WriteData(buf, 14);
}

void HardFault_Handler(void)
{
  led_Set(true, true);
  while(1) {}
}

int main(void)
{
	//SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

	int dummy = 0;
  FPUEnable();
  
	int_Init();
  //led_Init();
	//button_Init();
	//uart_init();
	//pcsr_Init();
	//can_setup();
		
	uart_test();
	
	return 0;
}
