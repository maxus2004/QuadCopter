//IMU connections
//A5 ---> SCK
//A6 ---> MISO
//A7 ---> MOSI
//B0 ---> CS1
//B1 ---> CS3
//B10 ---> CS2


#pragma once

#include <stdbool.h>
#include "math_extras.h"

extern int16_t GYRO_RAW[3];
extern int16_t ACC_RAW[3];
extern int16_t MAG_RAW[3];

extern float GYRO_DATA[3];
extern float ACC_DATA[3];
extern float MAG_DATA[3];
extern float FILTER_K;


void IMU_dataRead(void);
void IMU_init(void);
void IMU_readEuler(float fps, euler *orientation, vector3 *abs_accel, bool magnetometerEnabled);
void I2C_restart(void);
