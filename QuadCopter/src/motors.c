#include <stm32f1xx_ll_dma.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_bus.h>
#include <string.h>
#include <math.h>

#include "motors.h"

#include "shared.h"

uint16_t DSHOT_DMA_DATA[16];
uint16_t DSHOT_DMA_MASK = 0;
uint16_t motorPins[4];
uint16_t motorCount = 4;
uint16_t minThrottle = 0;
uint16_t maxThrottle = 0;

uint32_t prevPacket = 0;
uint32_t packetInterval = 100;     // 1 ms (1000 Hz)

static void motors_GPIO_init(GPIO_TypeDef* port, uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(port, &GPIO_InitStruct);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&port->BSRR);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&port->BRR);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&port->BRR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&DSHOT_DMA_MASK);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&DSHOT_DMA_DATA);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&DSHOT_DMA_MASK);
}
static void TIM1_Init(void)
{

	LL_TIM_InitTypeDef TIM_InitStruct;
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct;

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	TIM_InitStruct.Prescaler = 2;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 120;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = 0;
	LL_TIM_Init(TIM1, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM1);
	LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 45;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 90;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);
	TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
	TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
	TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	TIM_BDTRInitStruct.DeadTime = 0;
	TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
	TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
	TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
	LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}
static void DMA_Init(void)
{

/* Init with LL driver */
/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	/* TIM1 DMA Init */

	/* TIM1_UP Init */
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_HALFWORD);

	/* TIM1_CH1 Init */
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

	/* TIM1_CH2 Init */
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);

}

void sendDSHOT(uint16_t values[])
{
	memset(DSHOT_DMA_DATA, 0, sizeof(DSHOT_DMA_DATA));
	for (int i = 0; i < motorCount; i++)
	{
		uint16_t data = (uint16_t)(values[i] << 5);

		uint8_t XOR = ((data >> 4) ^ (data >> 8) ^ (data >> 12)) % 16;
		data = data | XOR;

		for (int j = 0; j < 16; j++)
		{
			DSHOT_DMA_DATA[15 - j] |= ((data >> j) % 2) ? 0 : motorPins[i];
		}
	}



	LL_TIM_DisableDMAReq_UPDATE(TIM1);
	LL_TIM_DisableDMAReq_CC1(TIM1);
	LL_TIM_DisableDMAReq_CC2(TIM1);
	LL_TIM_DisableCounter(TIM1);
	LL_TIM_SetCounter(TIM1, LL_TIM_GetAutoReload(TIM1));

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, 16);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 16);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 16);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

	LL_TIM_EnableDMAReq_UPDATE(TIM1);
	LL_TIM_EnableDMAReq_CC1(TIM1);
	LL_TIM_EnableDMAReq_CC2(TIM1);
	LL_TIM_EnableCounter(TIM1);

}

float filteredValues[4];
void setMotors(float m[])
{
	for (int i = 0; i < 4; i++)
	{
		if (isnanf(m[i])) continue;

		if (m[i] < 0)
		{
			m[i] = 0;
		} else if (m[i] > 1)
		{
			m[i] = 1;
		}
		filteredValues[i] = 0.8f * filteredValues[i] + 0.2f * m[i];
	}
	if (time - prevPacket > packetInterval)
	{
		uint16_t values[4];
		for (int i = 0; i < 4; i++)
		{
			values[i] = (uint16_t)(sqrtf(filteredValues[i]) * (float)(maxThrottle - minThrottle)) + minThrottle;
		}
		sendDSHOT(values);
		prevPacket = time;
	}
}

void stopMotors()
{
	if (time - prevPacket > packetInterval)
	{
		prevPacket = time;
		uint16_t values[] = { 0, 0, 0, 0 };
		memset(filteredValues, 0, sizeof(float) * 4);

		sendDSHOT(values);
	}
}

void setupMotors(GPIO_TypeDef* port, uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4, uint16_t _minThrottle, uint16_t _maxThrottle)
{
	DMA_Init();
	TIM1_Init();

	minThrottle = _minThrottle;
	maxThrottle = _maxThrottle;

	m1 = (uint16_t)(1 << m1);
	m2 = (uint16_t)(1 << m2);
	m3 = (uint16_t)(1 << m3);
	m4 = (uint16_t)(1 << m4);

	motorPins[0] = m1;
	motorPins[1] = m2;
	motorPins[2] = m3;
	motorPins[3] = m4;

	DSHOT_DMA_MASK = m1 | m2 | m3 | m4;

	motors_GPIO_init(port, m1, m2, m3, m4);

}

