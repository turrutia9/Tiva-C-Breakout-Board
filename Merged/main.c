#include <TM4C1294NCPDT.h> //structs defined here
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "can.h"
#include "list.h"

//uart includes
#include "uart_setup.h"

#include "main.h"

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

void uart_echo(){
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

//for reference -- configures the UART bases from register level
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

int binarySearch(int *list, int listLength, int item)
{
    int index=listLength/2;
    int max=listLength;
    int min=0;
    int oldIndex=0;
    bool found=false;

    while (index!=oldIndex)
    {
        if (list[index]==item)
        {
            found=true;
            break;
        }
        else
        {
            if (list[index]<item) //go bigger
            {
                min=index;
                oldIndex=index;
                index=(max-min)/2+min;
            }
            else //list[index] is bigger; so go smaller index
            {
                max=index;
                oldIndex=index;
                index=(max-min)/2+min;
            }

            if (index==oldIndex || index>listLength) //item isnt in list;
            {
                found=false;
                break;
            }
        }
    }

    if (!found)
    {
        index=-1;
    }

    return index;   
}

/*
If on ID gets handled here
*/
States updateState(CommandPacket *cmd)
{
	static States currentState=PassThrough; //init uC currentState to passthrough
	static States desiredState=PassThrough; //init the desired state to PT; get this from parse
	
	//parse for cmd - call parse function
	static uint32_t desiredID; //used for command on ID; get this from the parse function
	tCANMsgObject* bufferMsg; //used for extracting the buffer contents
	
	
	
	if (currentState == desiredState) //not waiting to find a specific ID
	{
		//parseUARTMsg(UARTmessage, &desiredID);
		switch (cmd->UARTMsg)
		{
			case 0x1: //start injection
				desiredState=Injection;
				currentState=Injection; //set the currentState to injection mode
				cmd->noise=false;
			
				break;
			
			case 0x2: //stop injection
				desiredState=PassThrough;
				currentState=PassThrough;
				
				break;
			
			case 0x3: //sub msg data immediate
				desiredState=Substitute;
				currentState=Substitute;
				//set the msg data here
				
				break;
			
			case 0x4: //sub msg data immediate w/ noise
				desiredState=Substitute;
				currentState=Substitute;
				
				//set msg data w/ noise here
				
				break;
			
			case 0x5: //inject immedate w/ id
				desiredState=Injection;
				currentState=Injection;
				
				break;
			
			case 0x6: //start UART comms
				desiredState=PassThrough;
				currentState=PassThrough;
			
				break;
			
			case 0x7: //stop UART comms
				desiredState=Disconnected;
				currentState=Disconnected;
				break;
			
			case 0x8: //stop injection on specific ID
				desiredState=PassThrough;
			
				for (int i=0; i<fifoLength; i++)
				{
					CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false); //get a message from the buffer
					if ((bufferMsg+i)->ui32MsgID == desiredID) //found the ID
					{
							currentState=PassThrough; //desired state achieved
							break;
					}
					CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
				}

				
				break;
			
			case 0x9: //start injection on ID
				desiredState=Injection;
			
				for (int i=0; i<fifoLength; i++)
				{
					CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false);	
					if ((bufferMsg+i)->ui32MsgID == desiredID) //found the ID
					{
							currentState=Injection; //desired state achieved
							break;
					}
					CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
				}				
				break;
			
			case 0xA: //start pass through
				desiredState=PassThrough;
				currentState=PassThrough;
				break;
			
			case 0xB: //stop pass through
				desiredState=Disconnected;
				currentState=Disconnected;			
				break;
			
			case 0xC: //inject immediate w/ noise
				desiredState=Injection;
				currentState=Injection;
			
			
				break;
			
			case 0xD: //clear injection on ID
				desiredState=Clear;
				for (int i=0; i<fifoLength; i++)
				{
					CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false);	
					if ((bufferMsg+i)->ui32MsgID == desiredID) //found the ID
					{
							currentState=Clear; //desired state achieved
							break;
					}
					CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
				}
			
				break;
			
			case 0xE: //start drop on ID			
				desiredState=Drop;
			
				for (int i=0; i<fifoLength; i++)
				{
					CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false);	
					if ((bufferMsg+i)->ui32MsgID == desiredID) //found the ID
					{
							currentState=Drop; //desired state achieved
							break;
					}
					CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
				}
				break;
			
			case 0xF: //stop drop on ID			
				desiredState=PassThrough;
			
				for (int i=0; i<fifoLength; i++)
				{
					CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false);	
					if ((bufferMsg+i)->ui32MsgID == desiredID) //found the ID
					{
							currentState=PassThrough; //desired state achieved
							break;
					}
					CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
						
				}
				break;
			
			case 0x10: //start drop once on ID	
				desiredState=DropOnce;
			
				for (int i=0; i<fifoLength; i++)
				{
					CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false);	
					if ((bufferMsg+i)->ui32MsgID == desiredID) //found the ID; don't change the state
					{
						currentState=DropOnce;
						break;
					}
					CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
				}
				break;
			
			case 0x11: //stop drop once on ID
				desiredState=PassThrough;
			
				for (int i=0; i<fifoLength; i++)
				{
					CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false);	
					if ((bufferMsg+i)->ui32MsgID == desiredID) //found the ID; don't change the state
					{
						currentState=PassThrough;
						break;
					}
					CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
				}
				break;
				
				
			return currentState; //indicate the desired state has been achieved 
		} //end of UART cmd switch/case
		
	} //if (currentState==desiredState)
  else //look for the ID
  {
    for (int i=0; i<fifoLength; i++)
    {
			CANMessageGet(RECEIVE_CAN_BASE, i, bufferMsg, false);	
      if (bufferMsg->ui32MsgID==desiredID) //found the desired id
      {
        currentState=desiredState;
				return currentState; //indicate the all state start conditions have been achieved 
      }
			CANMessageSet(SEND_CAN_BASE, i, bufferMsg, MSG_OBJ_TYPE_TX); //spit the message back out
    }
		return Waiting; //indicate still waiting to start the next state
  }
}

/*This only runs the code for each state, all conditions are assumed to have been satisfied
when this function is called
*/
void executeState(States state, CommandPacket data)
{
	if (state==Injection)
	{
		CanPacket injectPacket;
		injectPacket.arbid=0x130; //set the object ID
	
		//Set the data
		injectPacket.data[0]=0xC0;
		injectPacket.data[1]=0xFF;
		injectPacket.data[2]=0xEE;
		injectPacket.data[3]=0xC0;
		injectPacket.data[4]=0xFF;
		injectPacket.data[5]=0xEE;
		injectPacket.data[6]=0xC0;
		injectPacket.data[7]=0xFF;
		
		if (data.noise==true)
		{
			uint16_t noise=getNoise();
			noise&=0x1; //bitmask first bit to isolate
			for (int i=0; i<8; i++)
			{
				
				injectPacket.data[i]^=noise;	
			}
		}
	
		can_Inject(&injectPacket, SEND_CAN_BASE); //inject data
	}
	else if (state==PassThrough)
	{
		//receive all messages from the buffer
		tCANMsgObject *receivedMsgs=SetReceiveAll(RECEIVE_CAN_BASE, fifoLength);
		
		for (int i=0; i<fifoLength; i++) //send them all out on the transmission CAN
		{
			CANMessageSet(SEND_CAN_BASE, i, &receivedMsgs[i], MSG_OBJ_TYPE_TX); //send the message
		}
		
		fifoLength=0; //reset to indicate empty fifo buffer
	}
	else if (state==Substitute)
	{
		if (data.noise==true)
		{
			uint8_t noise=(uint8_t)getNoise();
		}
	}
	else if (state==Drop)
	{
		tCANMsgObject msg;
		for (int i=0; i<fifoLength; i++)
		{
			CANMessageGet(RECEIVE_CAN_BASE, i, &msg, true);
		}
		//dont send them
		fifoLength=0; //reset the fifolength to indicate emtpy
	}
	else if (state==DropOnce)
	{
		tCANMsgObject msg;
		CANMessageGet(RECEIVE_CAN_BASE, 0, &msg, true);
	}
	else if (state==Clear)
	{
		tCANMsgObject msg;
		for (int i=0; i<FIFO_MAX_LENGTH; i++) //clear the transmission buffer
		{
			CANMessageGet(SEND_CAN_BASE, 0, &msg, false);	
		}
	}
}

uint16_t getNoise()
{
	//uses a linear feedback register
	//https://en.wikipedia.org/wiki/Linear-feedback_shift_register
	
	  uint16_t start = 0x53F0;  /* Any nonzero start state will work. */
    uint16_t lfsr = start;
    static unsigned period = 0;

    do
    {
        lfsr ^= lfsr >> 7;
        lfsr ^= lfsr << 9;
        lfsr ^= lfsr >> 13;
        ++period;
    }
    while (lfsr != start);

    return period;
}

int main(void)
{
	SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

	int dummy = 0;
  	FPUEnable();
  
	int_Init();
  	led_Init();
	button_Init();
	uart_init();
	//pcsr_Init();
	can_setup();

	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	uart_send((uint8_t *)"\033[2JEnter text: ", 16);

	while (SysTickValueGet() != 0){
		//check for message
		if (SysTickValueget() == 0){
			//if message is found, process
		}
		//FSM
		//else, reset systick
	}
}
