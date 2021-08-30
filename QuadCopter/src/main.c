#include <math.h>

#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include <string.h>

#include "SPI.h"
#include "shared.h"
#include "IMU.h"
#include "PID.h"
#include "motors.h"
#include "math_extras.h"
#include "barometer.h"
#include "radio.h"
#include "telemetry.h"
#include "sr04.h"

#define TOO_HIGH_THRUST_ON_STARTUP 1
#define TOO_HIGH_THRUST_CHANGE 2
int errorId;

volatile uint32_t time = 0;
float dTime = 0;
float fps = 0;
uint32_t prevLoopTime = 0;


uint32_t prevReceive = 0, prevRadioRestart = 0;

volatile uint8_t STATE = STARTING;

euler rotation = { 0, 0, 0 };
euler target = { 0, 0, 0 };
float targetAltitude = 0;
vector3 absAcceleration = { 0, 0, 0 };
float thrust = 0, prevThrust = 0;
float motors[] = { 0, 0, 0, 0 };
float motorsTarget[] = { 0, 0, 0, 0 };
euler torque = { 0, 0, 0 };
float pressure = 0, temperature = 0;
float sonarAlt = 0, prevSonarAlt = 0;
float barAlt = 0, prevBarAlt = 0;
float vSpeed = 0;
float altitude = 0, prevAltitude = 0;
#define SONAR 0
#define BAROMETER 1
int prevAltitideCalcMode = BAROMETER;
int altitideCalcMode = BAROMETER;
float altitudeKSonar = 0.002f;
float altitudeKBar = 0.0005f;
float vSpeedK = 0.0005f;
bool heightStabilizationOn = false;
bool magnetometerEnabled = false;


float hP = 0, hI = 0, hD = 0;

void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
	}
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1)
	{

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}
	//100000 ticks per second (every 10 microseconds)
	LL_InitTick(72000000, 100000);
	LL_SYSTICK_EnableIT();
	LL_SetSystemCoreClock(72000000);
}
void SysTick_Handler(void)
{
	time++;
}

//magnetometer calibration
int16_t mag_max[3];
int16_t max_min[3];
int16_t mag_cal[3];
void magnetometerCalibrate(void)
{
	for (int i = 0; i < 3; i++)
	{
		if (MAG_RAW[i] > mag_max[i])
		{
			mag_max[i] = MAG_RAW[i];
		}
		else if (MAG_RAW[i] < max_min[i])
		{
			max_min[i] = MAG_RAW[i];
		}
		mag_cal[i] = -(mag_max[i] + max_min[i]) / 2;
	}
}

void emergencyLoop() {
	stopMotors();
	LL_mDelay(100);
}
void startEmergencyLoop() {
	while (1) {
		emergencyLoop();
	}
}

void setAltitudeStabilisation(bool enabled) {
	heightStabilizationOn = enabled;
	if (enabled)
	{
		targetAltitude = altitude;
		hI = thrust;
	}
}
void radioReceiveSend() {
	if (radio_irq())
	{
		radio_processInterrupt();
	}

	if (newData())
	{
		bool newHeightStabilizationOn;
		float thrustChange;
		bool success = radio_receive((float)(time - prevReceive) / 100000.0f, &target.roll, &target.pitch, &target.yaw, &thrustChange, &newHeightStabilizationOn);
		if (success)
		{
			prevReceive = time;

			if (newHeightStabilizationOn != heightStabilizationOn)
			{
				setAltitudeStabilisation(newHeightStabilizationOn);
			}

			if (heightStabilizationOn)
			{
				targetAltitude += thrustChange;
			}
			else
			{
				thrust += thrustChange;
				if (thrust > 1) thrust = 1;
				else if (thrust < 0) thrust = 0;
			}

			if (STATE == AUTO_LANDING)STATE = FLYING;

			const s_telemetry telemetry =
			{
				rotation,
				target,
				absAcceleration,
				{ACC_DATA[0],ACC_DATA[1],ACC_DATA[2]},
				thrust,
				altitude,
				targetAltitude,
				STATE
			};
			radio_sendTelemetry(telemetry);
		}
		else
		{
			radio_startReceiving();
		}
	}
}
void calculateTime() {
	dTime = (float)(time - prevLoopTime) / 100000.0f;
	prevLoopTime = time;
	fps = 1 / dTime;
}
void readData() {
	IMU_readEuler(fps, &rotation, &absAcceleration, magnetometerEnabled);
	prevBarAlt = barAlt;
	barometer_update(&pressure, &barAlt, &temperature);
	prevSonarAlt = sonarAlt;
	sonarAlt = sr04_dist();
}
void calculateAltitude() {
	//calculate altitude
	prevAltitude = altitude;
	prevAltitideCalcMode = altitideCalcMode;

	//if (sonarAlt > 0 && sonarAlt < 2 && fabsf(rotation.pitch) < 15 && fabsf(rotation.roll) < 15)
	//	altitideCalcMode = SONAR;
	//else
	//	altitideCalcMode = BAROMETER;
	altitideCalcMode = BAROMETER;


	if (altitideCalcMode == SONAR)
	{
		if (prevAltitideCalcMode == BAROMETER)
		{
			altitude -= prevBarAlt - sonarAlt;
			if (heightStabilizationOn)
			{
				targetAltitude = targetAltitude - (prevAltitude - altitude);
			}
		}

		if (dTime > 0 && (prevSonarAlt > 0 && prevSonarAlt < 2))
		{
			vSpeed = (1 - vSpeedK) * (vSpeed + absAcceleration.z * dTime) + vSpeedK * (sonarAlt - prevSonarAlt) / dTime;
		}
		altitude = (1 - altitudeKSonar) * (altitude + vSpeed * dTime) + altitudeKSonar * sonarAlt;

	}
	else
	{
		if (prevAltitideCalcMode == SONAR)
		{
			altitude -= prevSonarAlt - barAlt;
			if (heightStabilizationOn)
			{
				targetAltitude = targetAltitude - (prevAltitude - altitude);
			}
		}

		if (dTime > 0)
		{
			vSpeed = (1 - vSpeedK) * (vSpeed + absAcceleration.z * dTime) + vSpeedK * (barAlt - prevBarAlt) / dTime;
		}
		altitude = (1 - altitudeKBar) * (altitude + vSpeed * dTime) + altitudeKBar * barAlt;

	}
}
void updateMotors() {
	PID_update(dTime, rotation, target, &torque);
	motors[0] = thrust - torque.roll + torque.pitch + torque.yaw;
	motors[1] = thrust + torque.roll + torque.pitch - torque.yaw;
	motors[2] = thrust + torque.roll - torque.pitch + torque.yaw;
	motors[3] = thrust - torque.roll - torque.pitch - torque.yaw;
	setMotors(motors);
}
void checkThrust() {
	if (thrust - prevThrust > 0.5f)
	{
		STATE = EMERGENCY;
		errorId = TOO_HIGH_THRUST_CHANGE;
		startEmergencyLoop();
	}
}
void thrustPID() {
	float Kp = 0.2f;
	float Ki = 0.0f;
	float Kd = 0.1f;

	float e = targetAltitude - altitude;
	hP = Kp * e;
	hI = hI + Ki * dTime * (e);
	hD = Kd * -vSpeed;

	thrust = hP + hI + hD;
	if (thrust > 1)thrust = 1;
	if (thrust < 0)thrust = 0;
}

void startingLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	if (time > 500000)
	{
		FILTER_K = 0.1f;
		target.yaw = rotation.yaw;
		STATE = LANDED;
		prevReceive = time;
		radio_startReceiving();
	}
	calculateAltitude();
	checkThrust();
}
void landedLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	radioReceiveSend();
	calculateAltitude();
	if (thrust > 0)
	{
		if (thrust > 0.2f)
		{
			STATE = EMERGENCY;
			errorId = TOO_HIGH_THRUST_ON_STARTUP;
			return;
		}
		STATE = FLYING;
		target.yaw = rotation.yaw;
		PID_doIntegration = true;
		pid_values.i.yaw = 0;
		pid_values.i.pitch = 0;
		pid_values.i.roll = 0;
	}
	else
	{
		stopMotors();
	}
	checkThrust();
}
void flyingLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	radioReceiveSend();
	if (time - prevReceive > 50000)
	{
		STATE = AUTO_LANDING;
	}
	calculateAltitude();
	if (heightStabilizationOn)
	{
		thrustPID();
	}
	if (thrust <= 0)
	{
		PID_doIntegration = false;
		targetAltitude = 0;
		STATE = LANDED;
	}
	checkThrust();
	updateMotors();
}
void autolandingLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	radioReceiveSend();
	calculateAltitude();
	if (heightStabilizationOn)
	{
		targetAltitude -= dTime * 2;
		thrustPID();
	}
	else 
	{
		thrust -= dTime * 0.2f;
	}
	target.pitch = 0;
	target.roll = 0;

	if (thrust <= 0)
	{
		PID_doIntegration = false;
		targetAltitude = 0;
		STATE = LANDED;
	}

	checkThrust();
	updateMotors();
}

void startup() {
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	NVIC_SetPriorityGrouping(5);
	LL_GPIO_AF_Remap_SWJ_NOJTAG();
	SystemClock_Config();

	LL_mDelay(10000);

	SPI_init();
	radio_init();
	IMU_init();
	barometer_init();
	sr04_setup();

	setupMotors(GPIOB, 7, 6, 5, 8, 48, 2000);
	for (int i = 0; i < 100; i++)
	{
		stopMotors();
		LL_mDelay(1000);
	}

	STATE = STARTING;
	prevLoopTime = time;
	FILTER_K = 1;
}

int main(void)
{
	startup();

	while (1)
	{
		switch (STATE) {
		case STARTING:
			startingLoop();
			break;
		case LANDED:
			landedLoop();
			break;
		case FLYING:
			flyingLoop();
			break;
		case AUTO_LANDING:
			autolandingLoop();
			break;
		case EMERGENCY:
			startEmergencyLoop();
			break;
		}
	}
}
