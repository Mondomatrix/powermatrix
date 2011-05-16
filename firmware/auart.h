#ifndef RBG_AUART
#define RBG_AUART

#include <stdint.h>
#include <avr/io.h>
#include "avrlibdefs.h"

#define RS485ControlPort	PORTD
#define RS485ControlDir		DDRD
#define RS485ControlPin		2

#define AUARTPort	PORTD
#define AUARTDir	DDRD
#define AUARTTXPin	1
#define AUARTRXPin	0

void AUARTSetup();

#endif
