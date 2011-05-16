#include "mcp23s17.h"
#include <stdint.h>

void MCP23S17SetupAll(void)
{
	MCP23S17WriteRegister(0, MCP23S17_IOCON, 0b00101000);
//	MCP23S17WriteRegister(1, MCP23S17_IOCON, 0b00101000);
//	MCP23S17WriteRegister(2, MCP23S17_IOCON, 0b00101000);
//	MCP23S17WriteRegister(3, MCP23S17_IOCON, 0b00101000);
}

void MCP23S17Setup(uint8_t address)
{
	MCP23S17WriteRegister(address, MCP23S17_IODIRA, 0);
	MCP23S17WriteRegister(address, MCP23S17_IODIRB, 0);
}

void MCP23S17WriteRegister(uint8_t address, uint8_t reg, uint8_t data)
{
	uint8_t control;
	
	address &= 0b00000111;
	address <<= 1;
	control = 0b01000000 | address;
	SPISSOff();
	SPIWriteByte(control);
	SPIWriteByte(reg);
	SPIWriteByte(data);
	SPISSOn();
}

uint8_t MCP23S17ReadRegister(uint8_t address, uint8_t reg)
{
	uint8_t control;
	uint8_t dataBuff;
	
	address &= 0b00000111;
	address <<= 1;
	control = 0b01000001 | address;
	SPISSOff();
	SPIWriteByte(control);
	SPIWriteByte(reg);
	dataBuff = SPIReadByte();
	SPISSOn();
	return dataBuff;
}

void MCP23S17WriteIO(uint8_t address, uint8_t Aport, uint8_t Bport)
{
	uint8_t control;
	
	address &= 0b00000111;
	address <<= 1;
	control = 0b01000000 | address;
	SPISSOff();
	SPIWriteByte(control);
	SPIWriteByte(MCP23S17_GPIOA);
	SPIWriteByte(Aport);
	SPIWriteByte(Bport);
	//SPISSOn();
}

void MCP23S17QuickWriteIO(uint8_t Aport, uint8_t Bport)
{
	SPIWriteByte(Aport);
	SPIWriteByte(Bport);
}
