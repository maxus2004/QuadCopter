#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stdbool.h>
#include "radio.h"
#include "SPI.h"
#include "stm32f1xx_ll_utils.h"
#include "string.h"

volatile bool newPacket = false;
volatile bool receiving = false;
volatile bool sending = false;

static void cs1(void)
{
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
}
static void cs0(void)
{
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
}
bool radio_irq(void)
{
	return (LL_GPIO_ReadInputPort(GPIOA) & (1 << 4)) == 0;
}

void radio_writeReg(uint8_t reg, uint8_t data)
{
	cs0();
	SPI_SendReceive(0x80 | reg);
	SPI_SendReceive(data);
	cs1();
}

void radio_writeBurst(uint8_t reg, uint8_t* data, uint8_t length)
{
	cs0();
	SPI_SendReceive(0x80 | reg);
	for (int i = 0; i < length; i++)
	{
		SPI_SendReceive(data[i]);
	}
	cs1();
}

void radio_readBurst(uint8_t reg, uint8_t* data, uint8_t length)
{
	cs0();
	SPI_SendReceive(reg);
	for (int i = 0; i < length; i++)
	{
		data[i] = SPI_SendReceive(0xff);
	}
	cs1();
}

uint8_t radio_readReg(uint8_t reg)
{
	cs0();
	SPI_SendReceive(reg);
	uint8_t data = SPI_SendReceive(0xff);
	cs1();
	return data;
}

uint8_t getInterruptStatus(void)
{
	uint8_t data[2];
	radio_readBurst(0x03, data, 2);
	return data[0];
}

bool newData(void)
{
	return newPacket;
}


uint8_t value;
void radio_init(void)
{
	//CS pin
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//IRQ pin
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);
	//software reset
	cs0();
	LL_mDelay(15);
	radio_readReg(0x03);
	radio_readReg(0x04);
	radio_writeReg(0x07, 0x80);
	while (!radio_irq()) {}
	radio_readReg(0x03);
	radio_readReg(0x04);

	//set parameters from excel sheet
	//----------------------------
	//receiving frequency parameters
	radio_writeReg(0x1C, 0x82);
	radio_writeReg(0x1D, 0x40);
	radio_writeReg(0x20, 0x5E);
	radio_writeReg(0x21, 0x01);
	radio_writeReg(0x22, 0x5D);
	radio_writeReg(0x23, 0x86);
	radio_writeReg(0x24, 0x03);
	radio_writeReg(0x25, 0x7E);
	//enable packet handler and configure crc
	radio_writeReg(0x30, 0x8C);
	//disable header filtering
	radio_writeReg(0x32, 0x8C);
	//disable header, 2 bytes sync word
	radio_writeReg(0x33, 0x02);
	//preamble length
	radio_writeReg(0x34, 0x08);
	//preamble detection threshold 20 bits
	radio_writeReg(0x35, 0x24);
	//set sync word 0xABCD
	radio_writeReg(0x36, 0x2D);
	radio_writeReg(0x37, 0xD4);
	//
	radio_writeReg(0x38, 0x00);
	radio_writeReg(0x39, 0x00);
	radio_writeReg(0x3A, 0x00);
	radio_writeReg(0x3B, 0x00);
	radio_writeReg(0x3C, 0x00);
	radio_writeReg(0x3D, 0x00);
	radio_writeReg(0x3E, 0x00);
	radio_writeReg(0x3F, 0x00);
	radio_writeReg(0x40, 0x00);
	radio_writeReg(0x41, 0x00);
	radio_writeReg(0x42, 0x00);
	radio_writeReg(0x43, 0xFF);
	radio_writeReg(0x44, 0xFF);
	radio_writeReg(0x45, 0xFF);
	radio_writeReg(0x46, 0xFF);
	radio_writeReg(0x56, 0x00);
	//baud rate
	radio_writeReg(0x6E, 0x20);
	radio_writeReg(0x6F, 0xC5);
	radio_writeReg(0x70, 0x0C);
	//FIFO enabled and GFSK modulation
	radio_writeReg(0x71, 0x63);
	//frequency deviation
	radio_writeReg(0x72, 0x50);
	//carrier frequency
	radio_writeReg(0x75, 0x53);
	radio_writeReg(0x76, 0x4E);
	radio_writeReg(0x77, 0x20);
	//---------------------------------

	//set SGI bit
	radio_writeReg(0x69, 0x60);
	//tx power +20dBm
	radio_writeReg(0x6D, 0b00000111);
	//oscillator capacitive load
	radio_writeReg(0x09, 0xD7);
	//configure GPIO for RF switch
	radio_writeReg(0x0B, 0x12);
	radio_writeReg(0x0C, 0x15);
	//enable data out
	radio_writeReg(0x0D, 0b00010100);

	//enable packet sent and received interrupts
	radio_writeReg(0x05, 0b00000111);
	radio_writeReg(0x06, 0b00000000);
	radio_readReg(0x03);
	radio_readReg(0x04);

	//set mode to ready
	radio_writeReg(0x07, 0b00000001);

	value = radio_readReg(0x09);
}

volatile int sentpackets, crcerrors, validpackets;

void radio_processInterrupt(void)
{
	uint8_t status = getInterruptStatus();

	//packet received
	if (status & 1 << 1)
	{
		validpackets++;
		newPacket = true;
		receiving = false;
	}
	//crc error
	if (status & 1 << 0)
	{
		receiving = false;
		crcerrors++;
		radio_startReceiving();
	}
	//packet sent
	if (status & 1 << 2)
	{
		sentpackets++;
		sending = false;
		radio_startReceiving();
	}
}

void radio_sendPacket(void* data, uint8_t length)
{
	if (receiving || sending) return;
	sending = true;
	//set packet length
	radio_writeReg(0x3E, length);
	//fill FIFO
	radio_writeBurst(0x7F, data, length);
	//send packet
	radio_writeReg(0x07, 0b00001001);
}

uint8_t radio_readPacket(uint8_t* data)
{
	//read packet length
	uint8_t length = radio_readReg(0x4B);
	//read FIFO
	radio_readBurst(0x7F, data, length);
	return length;
}

void radio_startReceiving(void)
{
	if (sending) return;
	receiving = true;
	radio_writeReg(0x07, 0b00000101);
}

bool radio_receive(float dTime, float* tx, float* ty, float* tz, float* th, bool* stabilization)
{
	newPacket = false;
	uint8_t data[64];
	radio_readPacket(data);

	if (data[0] != 66)
	{
		return false;
	}

	float joysticks[4];
	memcpy(joysticks, data + 1, 16);

	if (dTime > 0.2f) dTime = 0.2f;

	if (joysticks[1] < -0.4f)joysticks[1] += 0.4f;
	else if (joysticks[1] > 0.4f)joysticks[1] -= 0.4f;
	else joysticks[1] = 0;

	if (joysticks[0] < -0.4f)joysticks[0] += 0.4f;
	else if (joysticks[0] > 0.4f)joysticks[0] -= 0.4f;
	else joysticks[0] = 0;

	if (joysticks[2] < -0.15f)joysticks[2] += 0.15f;
	else if (joysticks[2] > 0.15f)joysticks[2] -= 0.15f;
	else joysticks[2] = 0;

	if (joysticks[3] < -0.15f)joysticks[3] += 0.15f;
	else if (joysticks[3] > 0.15f)joysticks[3] -= 0.15f;
	else joysticks[3] = 0;

	float ntx = joysticks[3] * -20;
	float nty = joysticks[2] * -20;
	float thv = joysticks[1] * 1.5f;
	float tzv = joysticks[0] * -180;

	float nth = *th + thv * dTime;
	float ntz = *tz + tzv * dTime;
	if (ntz > 180)ntz = -180;
	if (ntz < -180)ntz = 180;

	*stabilization = (bool)data[17];

	*tx = ntx;
	*ty = nty;
	*tz = ntz;
	*th = nth;

	return true;
}

void radio_sendTelemetry(s_telemetry telemetry)
{
	uint8_t data[sizeof(s_telemetry) + 1];

	data[0] = 77;
	memcpy(data + 1, &telemetry, sizeof(s_telemetry));
	radio_sendPacket(data, sizeof(data));
}
