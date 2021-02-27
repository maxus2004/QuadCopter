//radio connections
//A0 ---> CS
//A5 ---> SCK
//A6 ---> MISO
//A7 ---> MOSI

#pragma once
#include <stdbool.h>
#include <stm32f1xx.h>
#include "telemetry.h"

void radio_processInterrupt(void);
bool radio_irq(void);
bool radio_receive(float dTime, float* tx, float* ty, float* tz, float* th, bool* stabilization);
void radio_init(void);
bool newData(void);
void radio_sendPacket(void* data, uint8_t length);
uint8_t radio_readPacket(uint8_t* data);
void radio_startReceiving(void);
void radio_sendTelemetry(s_telemetry telemetry);
