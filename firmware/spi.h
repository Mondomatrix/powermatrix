#ifndef RBGSPI
#define RBGSPI

#include <stdint.h>
#include "avrlibdefs.h"

#define PORT_SPI    PORTB
#define PORT_SS		2
#define DDR_SPI     DDRB
#define DD_MISO     DDB4
#define DD_MOSI     DDB3
#define DD_SS       DDB2
#define DD_SCK      DDB5

void SPISetup(void);
void SPISSOn(void);
void SPISSOff(void);
void SPIWriteByte(uint8_t dataByte);
uint8_t SPIReadByte(void);
uint8_t SPIReadWriteByte(uint8_t dataByte);
void SPIWriteArray(uint8_t* dataArray, uint8_t len);
void SPIReadArray(uint8_t* targetArray, uint8_t len);
void SPIReadWriteArray(uint8_t* dataArray, uint8_t* targetArray, uint8_t len);

#endif
