//uart includes

#include <inc\hw_uart.h>
#include <driverlib\uart.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <inc\hw_ints.h>
#include <inc/hw_memmap.h>

#define FUNCTION_LOG    0x00
#define FUNCTION_FILTER 0x01
#define FUNCTION_RESET  0x02

void uart_init();
void get_interrupt();
void uart_send(const uint8_t* buffer, uint32_t count);

void uart_test();

uint32_t g_ui32SysClock;