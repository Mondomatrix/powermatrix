#include "auart.h"
#include <stdint.h>
#include "avrlibdefs.h"

void AUARTSetup()
{
	UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
	UCSR0B = ((1 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02) | (0 << RXB80) | (0 << TXB80));
	UCSR0C = ((0 << UMSEL01) | (0 << UMSEL00) | (0 << UPM01) | (0 << UPM00) | (0 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0));
	UBRR0H = 0;
	UBRR0L = 21;

/*	//set baud rate
	UBRR0H = ((F_CPU / 16 + 38400 / 2) / 38400 - 1) >> 8;
	UBRR0L = ((F_CPU / 16 + 38400 / 2) / 38400 - 1);
	
    // reset config for UART0
	UCSR0A = 0;
	UCSR0B = 0;
	UCSR0C = 0;
	
	//configure UART0
    UCSR0B = _BV(RXEN0)|_BV(TXEN0)|_BV(UCSZ02);
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);	
*/
	RS485ControlDir &= ~((1 << RS485ControlPin));
	RS485ControlDir |= ((1 << RS485ControlPin));
	cbi(RS485ControlPort, RS485ControlPin);
}
