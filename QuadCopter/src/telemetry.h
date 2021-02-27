#pragma once
#include "math_extras.h"

typedef struct
{
	euler rotation;
	euler target_rotation;
	float thrust;
	float altitude;
	float target_altitude;
	float sonarAlt;
} s_telemetry;
