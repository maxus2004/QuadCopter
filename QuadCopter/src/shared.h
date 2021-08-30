#pragma once

#define STARTING 0
#define LANDED 1
#define FLYING 2
#define AUTO_LANDING 3
#define EMERGENCY 4
extern volatile uint8_t STATE;
extern volatile uint32_t time;
