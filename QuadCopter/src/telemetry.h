#pragma once
#include "math_extras.h"

typedef struct
{
	euler rotation;
	euler target_rotation;
	vector3 global_acceleration;
	vector3 acceleration;
	float thrust;
	float altitude;
	float target_altitude;
	uint8_t state;
} __attribute__((packed)) s_telemetry;
