#include <stm32f1xx_ll_spi.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_utils.h>
#include <math.h>

#include "filter.h"
#include "IMU.h"

#include <stdbool.h>

#include "math_extras.h"
#include "SPI.h"

#define ROLL_OFFSET (-0.8f)
#define PITCH_OFFSET (0.2f)
#define YAW_OFFSET 0.0f
#define GYRO_MUL 0.00109f
#define ACC_MUL 0.0195f
#define MAG_MUL 0.00512f
float FILTER_K = 0.1f;

#define CS_ACC LL_GPIO_PIN_0
#define CS_GYRO LL_GPIO_PIN_10
#define CS_MAG LL_GPIO_PIN_1

int16_t MAG_CAL[3] = { -47,15,64};
int16_t ACC_CAL[3] = { 0, 0, 15 };
int16_t GYRO_CAL[3] = { 0, 0, 0 };



int16_t GYRO_RAW[3];
int16_t ACC_RAW[3];
int16_t MAG_RAW[3];
float GYRO_DATA[3];
float ACC_DATA[3];
float MAG_DATA[3];

static void cs1(uint32_t pin)
{
	LL_GPIO_SetOutputPin(GPIOB, pin);
}
static void cs0(uint32_t pin)
{
	LL_GPIO_ResetOutputPin(GPIOB, pin);
}
uint8_t IMU_readReg(uint32_t cs_pin, uint8_t reg)
{
	cs0(cs_pin);
	SPI_SendReceive(0x80 | reg);
	uint8_t data = SPI_SendReceive(0xff);
	cs1(cs_pin);
	return data;
}
void IMU_writeReg(uint32_t cs_pin, uint8_t reg, uint8_t data)
{
	cs0(cs_pin);
	SPI_SendReceive(reg);
	SPI_SendReceive(data);
	cs1(cs_pin);
}

void IMU_dataRead()
{
//read accelerometer
	if ((IMU_readReg(CS_ACC, 0x0e) & 0b01111111) > 0)
	{
		cs0(CS_ACC);
		SPI_SendReceive(0x80 | 0x3f);
		for (uint8_t i = 0; i < 6; i++)
		{
			*((uint8_t*)ACC_RAW + i) = SPI_SendReceive(0xff);
		}
		cs1(CS_ACC);
	}
	//read gyroscope
	if ((IMU_readReg(CS_GYRO, 0x0e) & 0b01111111) > 0)
	{
		cs0(CS_GYRO);
		SPI_SendReceive(0x80 | 0x3f);
		for (uint8_t i = 0; i < 6; i++)
		{
			*((uint8_t*)GYRO_RAW + i) = SPI_SendReceive(0xff);
		}
		cs1(CS_GYRO);
	}
	//read magnetometer
	if (IMU_readReg(CS_MAG, 0x48) & 0b00000001)
	{
		cs0(CS_MAG);
		SPI_SendReceive(0x80 | 0x42);
		for (uint8_t i = 0; i < 6; i++)
		{
			*((uint8_t*)MAG_RAW + i) = SPI_SendReceive(0xff);
		}
		cs1(CS_MAG);
		MAG_RAW[0] >>= 3;
		MAG_RAW[1] >>= 3;
		MAG_RAW[2] >>= 1;
	}

	for (uint8_t i = 0; i < 3; i++)
	{
		GYRO_DATA[i] = (float)(GYRO_RAW[i] + GYRO_CAL[i]) * GYRO_MUL;
		ACC_DATA[i] = (float)((ACC_RAW[i] >> 4) + ACC_CAL[i]) * ACC_MUL;
		MAG_DATA[i] = (float)(MAG_RAW[i] + MAG_CAL[i]) * MAG_MUL;
	}
}

void IMU_init(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = CS_ACC | CS_GYRO | CS_MAG;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	cs1(CS_ACC);
	cs1(CS_MAG);
	cs1(CS_GYRO);

	LL_mDelay(100);

	//gyroscope configuration
	IMU_writeReg(CS_ACC, 0x11, 0b00000000);            //   normal mode
	LL_mDelay(100);
	IMU_writeReg(CS_ACC, 0x0F, 0b00000000);           //   2000 d/s
	IMU_writeReg(CS_ACC, 0x10, 0b00000000);           //   2000 Hz
	IMU_writeReg(CS_ACC, 0x12, 0b00000000);           //   default sleep mode config
	IMU_writeReg(CS_ACC, 0x13, 0b00000000);           //   data filtered, shadowing enabled
	IMU_writeReg(CS_ACC, 0x34, 0b00000000);           //   WDT disabled, 4-wire SPI
	IMU_writeReg(CS_ACC, 0x36, 0b00000000);           //   OFC disabled
	IMU_writeReg(CS_ACC, 0x3E, 0b00000000);           //   FIFO bypass
	//accelerometer configuration
	IMU_writeReg(CS_ACC, 0x11, 0b00000000);            //   normal mode
	LL_mDelay(100);
	IMU_writeReg(CS_ACC, 0x0F, 0b00000101);           //   4 g
	IMU_writeReg(CS_ACC, 0x10, 0b00001111);           //   1000 Hz
	IMU_writeReg(CS_ACC, 0x12, 0b00000000);           //   default sleep mode config
	IMU_writeReg(CS_ACC, 0x13, 0b00000000);           //   data filtered, shadowing enabled
	IMU_writeReg(CS_ACC, 0x34, 0b00000000);           //   WDT disabled, 4-wire SPI
	IMU_writeReg(CS_ACC, 0x36, 0b00000000);           //   OFC disabled
	IMU_writeReg(CS_ACC, 0x3E, 0b00000000);           //   FIFO bypass
	//magnetometer configuration
	IMU_writeReg(CS_MAG, 0x4B, 0b00000001);           //   4-wire SPI, suspend mode disabled
	IMU_writeReg(CS_MAG, 0x4C, 0b00101000);           //   20 Hz, normal mode
	IMU_writeReg(CS_MAG, 0x51, (47 - 1) / 2);         //   47 repetitions on XY (high accuracy preset)
	IMU_writeReg(CS_MAG, 0x52, (83 - 1));             //   83 repetitions on Z  (high accuracy preset)
}



void quaternionToEuler(float q0, float q1, float q2, float q3, float* pitch, float* roll, float* yaw)
{
	*roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 57.29f;
	*pitch = asinf(2 * (q0 * q2 - q3 * q1)) * 57.29f;
	*yaw = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 57.29f;
}




void IMU_readEuler(float fps, euler* orient, vector3* abs_accel, bool magnetometerEnabled)
{
	IMU_dataRead();

	if (fps > 0)
	{
		Madgwick_setKoeff(fps, FILTER_K);
		if (magnetometerEnabled)
			Madgwick_update(GYRO_DATA[0], GYRO_DATA[1], GYRO_DATA[2], ACC_DATA[0], ACC_DATA[1], ACC_DATA[2], -MAG_DATA[1], MAG_DATA[0], MAG_DATA[2]);
		else
			Madgwick_updateIMU(GYRO_DATA[0], GYRO_DATA[1], GYRO_DATA[2], ACC_DATA[0], ACC_DATA[1], ACC_DATA[2]);
	}

	float qw, qx, qy, qz;
	Madgwick_readQuaternions(&qw, &qx, &qy, &qz);
	quaternionToEuler(qw, qx, qy, qz, &orient->pitch, &orient->roll, &orient->yaw);
	orient->pitch += PITCH_OFFSET;
	orient->roll += ROLL_OFFSET;
	orient->yaw += YAW_OFFSET;

	float aw = 0, ax = ACC_DATA[0], ay = ACC_DATA[1], az = ACC_DATA[2];
	float qw1 = qw, qx1 = -qx, qy1 = -qy, qz1 = -qz;

	float bw = qw * aw - qx * ax - qy * ay - qz * az;
	float bx = qw * ax + qx * aw + qy * az - qz * ay;
	float by = qw * ay - qx * az + qy * aw + qz * ax;
	float bz = qw * az + qx * ay - qy * ax + qz * aw;

	float b1x = bw * qx1 + bx * qw1 + by * qz1 - bz * qy1;
	float b1y = bw * qy1 - bx * qz1 + by * qw1 + bz * qx1;
	float b1z = bw * qz1 + bx * qy1 - by * qx1 + bz * qw1;

	abs_accel->x = b1x;
	abs_accel->y = b1y;
	abs_accel->z = b1z - 9.8f;
}
