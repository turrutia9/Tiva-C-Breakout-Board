
#include "mainHeader.h"


int main(void)
{
	int dummy = 0;
	/*
  FPUEnable();
  
	int_Init();
  led_Init();
	button_Init();
	pcsr_Init();
  can_Init();
	*/
	SysTickEnable();
	
	uint32_t clockTicks=PERIOD * SystemCoreClock;
	SysTickPeriodSet(clockTicks); // 1/2 second period; param=period*clock rate
	//NVIC_ST_CURRENT
	

	while (SysTickValueGet() != 0)
	{
	}
	
	dummy=0;

	
	
	
	SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
	
	//transmit_uart_test();

	while(1){
		dummy++;
	}
	
}



/* Interrupt handler for CAN0
 * Called when receiving message on CAN0
 * Must clear the interrupt flags using CANIntClear()
 * Currently just passes the message back out on CAN1
 */
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

/* Function initializes both can modules
 * CAN0 is set to transmit messages
 * CAN1 is set to receive messages
 * This is only for temporary organization, get the full and tested ones from Tyler when finished
 */
void initCANModules(tCANBitClkParms *CANBitClk)
{
    //------------------ CAN0 ----------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0); // Enable the CAN0 module.

    // Wait for the CAN0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0))
    {
    }

    // Reset the state of all the message objects and the state of the CAN
    // module to a known state.
    CANInit(CAN0_BASE);
    CANEnable(CAN0_BASE); // Take the CAN0/CAN1 device out of INIT state.



    //------------------ CAN1 ------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1); // Enable the CAN1 module.

    // Wait for the CAN1 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN1))
    {
    }

    // Reset the state of all the message objects and the state of the CAN
    // module to a known state.
    CANInit(CAN1_BASE);


    // Configure the controller for 1 Mbit operation.
    CANBitTimingSet(CAN1_BASE, CANBitClk);

    // Take the CAN0/CAN1 device out of INIT state.
    CANEnable(CAN1_BASE);
} //end CANinit()

/* Function initializes GPIO port A
 * Make an object that contains: input, output, interrupt pins
 *
 */
void initGPIO_A(uint8_t inputPins, uint8_t outputPins)
{
    // Enable the GPIOA peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Wait for the GPIOA module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }

    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //GPIOIntRegister(GPIOA_AHB_BASE, (void)INT_GPIOA_TM4C129);

    // Initialize the GPIO pin configuration.
    // Set pins 2, 4, and 5 as input, SW controlled.
    GPIOPinTypeGPIOInput(GPIOA_AHB_BASE, inputPins);

    // Set pins 0 and 3 as output, SW controlled.
    GPIOPinTypeGPIOOutput(GPIOA_AHB_BASE, outputPins);

    // Make pins 0 and 1 rising edge triggered interrupts.
    GPIOIntTypeSet(GPIOA_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_RISING_EDGE);


    // Make pin 2 high level triggered interrupts.
    GPIOIntTypeSet(GPIOA_AHB_BASE, GPIO_PIN_2, GPIO_HIGH_LEVEL);

    // Enable the pin interrupts.
    GPIOIntEnable(GPIOA_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
}


//binary search to find a message of interest
//returns index in array if found or -1 if not found
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

CommandPacket parseUARTMsg(uint8_t *rawCmd)
{
	CommandPacket cmd;
	
	
	return cmd;
	//return a parsed command to then be handled in the FSM
} //end parseUARTMsg()


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


/*-------------------------------------------------------------------
													GIVEN DEFINITIONS
-------------------------------------------------------------------*/


void int_Init(void)
{
	IntPrioritySet(INT_GPIOJ_TM4C129,0x80);
	IntPrioritySet(INT_CAN0_TM4C129,0x00);
	IntPrioritySet(INT_CAN1_TM4C129,0x00);
}

void led_Init(void)
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

void uart_init(void)
{
	uint64_t baud = CAN_TARGET_BAUD; //might have to change this to 38400
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

//reference from this code
void pcsr_Init(void)
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
  uint64_t baud = CAN_TARGET_BAUD;
  
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
