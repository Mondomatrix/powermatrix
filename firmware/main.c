#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "avrlibdefs.h"
#include "spi.h"
#include "mcp23s17.h"
#include "auart.h"

#define GroupMask 0x00

//#define NORMOCRA 0x62

#define NORMOCRA 0x56
#define SERIAL_STATE_WAITING 0
#define SERIAL_STATE_COMMAND 1
#define SERIAL_STATE_DATA 2

volatile uint8_t MyAddress = 0;
volatile uint8_t CommandByte = 0;
volatile uint8_t DataBuffer[256];
volatile uint8_t DataCount = 0;
volatile uint8_t BytesReceived = 0;
volatile uint8_t ProcessCommand = 0;
volatile uint8_t timerIncrement = 1;

volatile uint8_t PWMVal[32];

//volatile uint8_t RoundCount = 0;
uint8_t RoundCount __asm__("r12") = 0;
volatile uint8_t Group = 0;
volatile uint8_t PulseState = 0;
volatile uint8_t DoIO = 0;
volatile uint8_t IOPulseState = 0;

void IOSetup(void);
void TimerSetup(void);
uint8_t SerialLengthFromCommand(uint8_t CmdByte);
void SerialStateMachine(uint8_t SerialByte, uint8_t NinthBit);
uint8_t GetMyAddress(void);

int main(void)
{
	uint8_t DataBufferA = 0;
	uint8_t DataBufferB = 0;
	uint8_t DataBufferC = 0;
	uint8_t DataBufferD = 0;
	uint8_t i, j, jdot, kdot;

	IOSetup();
	SPISetup();
	AUARTSetup();

	MCP23S17SetupAll();
	MCP23S17Setup(0);
	MCP23S17Setup(1);

	MyAddress = GetMyAddress();

	for(i=0;i<128;i++)
	{
		DataBuffer[i] = 0;
		DataBuffer[i+128] = 0;
	}

	for(i=0;i<32;i++)
	{
		PWMVal[i] = 129;
	}

	TimerSetup();

	while(1)
	{
		if(ProcessCommand == 1)
		{
			ProcessCommand = 0;
			//based on command byte, execute the function
			if(CommandByte == 1)
			{
				for(j=0;j<2;j++)
				{
					jdot = j * 16;
					PWMVal[0+jdot] = DataBuffer[3+jdot];
					PWMVal[1+jdot] = DataBuffer[2+jdot];
					PWMVal[2+jdot] = DataBuffer[1+jdot];
					PWMVal[3+jdot] = DataBuffer[0+jdot];
					PWMVal[4+jdot] = DataBuffer[4+jdot];
					PWMVal[5+jdot] = DataBuffer[5+jdot];
					PWMVal[6+jdot] = DataBuffer[6+jdot];
					PWMVal[7+jdot] = DataBuffer[7+jdot];
					PWMVal[8+jdot] = DataBuffer[11+jdot];
					PWMVal[9+jdot] = DataBuffer[10+jdot];
					PWMVal[10+jdot] = DataBuffer[9+jdot];
					PWMVal[11+jdot] = DataBuffer[8+jdot];
					PWMVal[12+jdot] = DataBuffer[12+jdot];
					PWMVal[13+jdot] = DataBuffer[13+jdot];
					PWMVal[14+jdot] = DataBuffer[14+jdot];
					PWMVal[15+jdot] = DataBuffer[15+jdot];
				}
			} else if(CommandByte == 2) {
				jdot = DataBuffer[0];
				kdot = jdot % 8;
				jdot = jdot - kdot;
				if(kdot > 4)
				{
					PWMVal[jdot+kdot] = DataBuffer[1];
				} else {
					PWMVal[jdot+(3-kdot)] = DataBuffer[1];
				}
			} else if(CommandByte == 3) {
				jdot = DataBuffer[0];
				switch (jdot) {
					case 0:
						timerIncrement = 1;
						break;
					case 1:
						timerIncrement = 2;
						break;
					case 2:
						timerIncrement = 4;
						break;
					case 3:
						timerIncrement = 8;
						break;
					case 4:
						timerIncrement = 16;
						break;
					case 5:
						timerIncrement = 32;
						break;
					case 6:
						timerIncrement = 64;
						break;
					case 7:
						timerIncrement = 128;
						break;
					default:
						timerIncrement = 1;
						break;
				}
			}
			CommandByte = 0;
			BytesReceived = 0;
			DataCount = 0;
		}

		if(DoIO == 1)
		{
			DoIO = 0;
			if(RoundCount == 0) {
				DataBufferA = 255;
				DataBufferB = 255;
				DataBufferC = 255;
				DataBufferD = 255;
			}

			if(RoundCount >= PWMVal[0]) cbi(DataBufferA, 0);
			if(RoundCount >= PWMVal[1]) cbi(DataBufferA, 1);
			if(RoundCount >= PWMVal[2]) cbi(DataBufferA, 2);
			if(RoundCount >= PWMVal[3]) cbi(DataBufferA, 3);
			if(RoundCount >= PWMVal[4]) cbi(DataBufferA, 4);
			if(RoundCount >= PWMVal[5]) cbi(DataBufferA, 5);
			if(RoundCount >= PWMVal[6]) cbi(DataBufferA, 6);
			if(RoundCount >= PWMVal[7]) cbi(DataBufferA, 7);

			if(RoundCount >= PWMVal[8]) cbi(DataBufferB, 0);
			if(RoundCount >= PWMVal[9]) cbi(DataBufferB, 1);
			if(RoundCount >= PWMVal[10]) cbi(DataBufferB, 2);
			if(RoundCount >= PWMVal[11]) cbi(DataBufferB, 3);
			if(RoundCount >= PWMVal[12]) cbi(DataBufferB, 4);
			if(RoundCount >= PWMVal[13]) cbi(DataBufferB, 5);
			if(RoundCount >= PWMVal[14]) cbi(DataBufferB, 6);
			if(RoundCount >= PWMVal[15]) cbi(DataBufferB, 7);

			SPISSOn();
			_delay_us(1);
			MCP23S17WriteIO(0, DataBufferA, DataBufferB);
			_delay_us(1);
			SPISSOff();
			
			if(RoundCount >= PWMVal[16]) cbi(DataBufferC, 0);
			if(RoundCount >= PWMVal[17]) cbi(DataBufferC, 1);
			if(RoundCount >= PWMVal[18]) cbi(DataBufferC, 2);
			if(RoundCount >= PWMVal[19]) cbi(DataBufferC, 3);
			if(RoundCount >= PWMVal[20]) cbi(DataBufferC, 4);
			if(RoundCount >= PWMVal[21]) cbi(DataBufferC, 5);
			if(RoundCount >= PWMVal[22]) cbi(DataBufferC, 6);
			if(RoundCount >= PWMVal[23]) cbi(DataBufferC, 7);

			if(RoundCount >= PWMVal[24]) cbi(DataBufferD, 0);
			if(RoundCount >= PWMVal[25]) cbi(DataBufferD, 1);
			if(RoundCount >= PWMVal[26]) cbi(DataBufferD, 2);
			if(RoundCount >= PWMVal[27]) cbi(DataBufferD, 3);
			if(RoundCount >= PWMVal[28]) cbi(DataBufferD, 4);
			if(RoundCount >= PWMVal[29]) cbi(DataBufferD, 5);
			if(RoundCount >= PWMVal[30]) cbi(DataBufferD, 6);
			if(RoundCount >= PWMVal[31]) cbi(DataBufferD, 7);
			
			SPISSOn();
			_delay_us(1);
			MCP23S17WriteIO(1, DataBufferC, DataBufferD);
			_delay_us(1);
			SPISSOff();			
		}
	}
	return 0;
}

uint8_t GetMyAddress(void)
{
	uint8_t addressBuffer = 0;
	//read the address, return it
	
	addressBuffer = PINC & 0x3F;
	addressBuffer |= ((PIND & 0x18) << 3);
	
	return addressBuffer;
}

void SerialStateMachine(uint8_t SerialByte, uint8_t NinthBit)
{
	static uint8_t SerialState = SERIAL_STATE_WAITING;
	
	switch(SerialState)
	{
		case SERIAL_STATE_WAITING:
			//make sure MPCM is set
			//check for address match
			if(SerialByte == MyAddress)
			{
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (0 << MPCM0));
				SerialState = SERIAL_STATE_COMMAND;
			} else {
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
				SerialState = SERIAL_STATE_WAITING;
			}
			//no match, resume
			//if match, clear MPCM, state = STATE_ADDRESS
			break;
		case SERIAL_STATE_COMMAND:
			//check ninth bit. if clear, continue, else, state = SERIAL_STATE_WAITING, set MPCM
			if(NinthBit != 0)
			{
				SerialState = SERIAL_STATE_WAITING;
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
			} else {
			//commandbyte = data
				CommandByte = SerialByte;
			//state = SERIAL_STATE_DATA
				SerialState = SERIAL_STATE_DATA;
			//set DataCount based on command
				DataCount = SerialLengthFromCommand(SerialByte);
				BytesReceived = 0;
			}
			break;
		case SERIAL_STATE_DATA:
			//check ninth bit. if clear, continue, else, state = SERIAL_STATE_WAITING, DataCount = 0
			if(NinthBit == 0)
			{
				DataBuffer[BytesReceived] = SerialByte;
			//DataBuffer[BytesReceived] = byte
				BytesReceived++;
				if(DataCount == BytesReceived)
				{
					ProcessCommand = 1;
					SerialState = SERIAL_STATE_WAITING;
					UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
				} else {
					SerialState = SERIAL_STATE_DATA;
				}
			} else {
				SerialState = SERIAL_STATE_WAITING;
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
			}
			//bytesreceived++
			//if datacount == bytesrecieved then call command processor, bytesreceived = 0, datacount = 0, state = SERIAL_STATE_WAITING, set MPCM
			//else state = SERIAL_STATE_DATA
			break;
		default:
			SerialState = SERIAL_STATE_WAITING; // also set MPCM
			break;
	}
}

uint8_t SerialLengthFromCommand(uint8_t CmdByte)
{
	//return proper data length for a command
	if(CmdByte == 1)
	{
		return 32;
	} else if(CmdByte == 2) {
		return 2;
	} else if(CmdByte == 3) {
		return 1;
	} else {
		return 0;
	}
}

void IOSetup(void)
{
	DDRC = 0;
	cbi(DDRD, 4);
}

void TimerSetup(void)
{
	TCCR0A = ((0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (0 << WGM00));
	TCCR0B = ((0 << FOC0A) | (0 << FOC0B) | (0 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00));
	TIMSK0 = ((0 << OCIE0B) | (1 << OCIE0A) | (0 << TOIE0));
	OCR0A = NORMOCRA;
	sei();
}

ISR(USART_RX_vect)
{
	uint8_t IntDataByte, IntNinthBit;
	IntDataByte = UDR0;
	IntNinthBit = UCSR0A & 0x01;
	SerialStateMachine(IntDataByte, IntNinthBit);
}

ISR(TIMER0_COMPA_vect)
{
	DoIO = 1;
	RoundCount+= timerIncrement;
}
