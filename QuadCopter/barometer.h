#pragma once

void barometer_init(void);
float barometer_temp(void);
float barometer_press(void);
float barometer_alt(void);
void barometer_update(float *press, float *alt, float *temp);
