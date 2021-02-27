#include "sr04.h"
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_exti.h>
#include "stm32f1xx_ll_utils.h"
#include "shared.h"
#include <stdbool.h>

static volatile uint32_t startTime = 0;
static volatile float dist = -1;
static volatile bool measuring = false;
static volatile bool pulse_sent = false;

void sr04_setup()
{
	//trig pin
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
	//echo pin
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);

	//interrupt
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE4);
	LL_EXTI_InitTypeDef EXTI_InitStruct = { 0 };
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);
	NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI4_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)
	{
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);

		if (!measuring)return;

		if (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_4))
		{
			pulse_sent = true;
			startTime = time;
		} else if(pulse_sent)
		{
			pulse_sent = false;
			float _dist = (time - startTime) * 0.001715f;
			if (_dist > 4)
			{
				_dist = -1;
			}
			dist = _dist;
			measuring = false;
		}
	}
}

float sr04_dist()
{
	if (!measuring && time-startTime>3000)
	{
		measuring = true;
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);
		LL_mDelay(1);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
	}

	return dist;
}