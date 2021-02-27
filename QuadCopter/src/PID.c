#include "PID.h"

#include <stdbool.h>

#include "math_extras.h"

//PARAMETERS--------------------------

PID_params_s pid_params = {
{ 0.015f, 0.015f, 0.03f },
 { 0.05f, 0.05f, 0.05f },
{ 0.003f, 0.003f, 0.0f },
 { 0.5f, 0.5f, 0.4f },
 { -0.5f, -0.5f, -0.4f },
 { 0.05f, 0.05f, 0.0f },
 { -0.05f, -0.05f, 0.0f }
};

//-----------------------------------

PID_values_s pid_values = {
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 }
};

static PID_values_s* pid = &pid_values;

bool PID_doIntegration = false;

void PID_resetI(void)
{
	pid->i.yaw = 0;
	pid->i.roll = 0;
	pid->i.pitch = 0;
}

void PID_update(float dTime, euler rotation, euler target, euler* torque)
{
	//error
	pid->E.yaw = target.yaw - rotation.yaw;
	pid->E.pitch = target.pitch - rotation.pitch;
	pid->E.roll = target.roll - rotation.roll;
	if (pid->E.yaw > 180)
	{
		pid->E.yaw -= 360;
	} else if (pid->E.yaw < -180)
	{
		pid->E.yaw += 360;
	}
	
	//proportional
	pid->p.roll = pid_params.KP.roll * pid->E.roll;
	pid->p.pitch = pid_params.KP.pitch * pid->E.pitch;
	pid->p.yaw = pid_params.KP.yaw * pid->E.yaw;

	//integral
	if (PID_doIntegration)
	{
		pid->i.roll = pid->i.roll + pid_params.KI.roll * dTime * (pid->E.roll);
		if (pid->i.roll > pid_params.maxI.roll) pid->i.roll = pid_params.maxI.roll;
		if (pid->i.roll < pid_params.minI.roll) pid->i.roll = pid_params.minI.roll;
		pid->i.pitch = pid->i.pitch + pid_params.KI.pitch * dTime * (pid->E.pitch);
		if (pid->i.pitch > pid_params.maxI.pitch) pid->i.pitch = pid_params.maxI.pitch;
		if (pid->i.pitch < pid_params.minI.pitch) pid->i.pitch = pid_params.minI.pitch;
		pid->i.yaw = pid->i.yaw + pid_params.KI.yaw * dTime * (pid->E.yaw);
		if (pid->i.yaw > pid_params.maxI.yaw) pid->i.yaw = pid_params.maxI.yaw;
		if (pid->i.yaw < pid_params.minI.yaw) pid->i.yaw = pid_params.minI.yaw;
	}
	//derivative
	pid->d.roll = pid_params.KD.roll * (-rotation.roll + pid->prev.roll) / dTime;
	pid->d.pitch = pid_params.KD.pitch * (-rotation.pitch + pid->prev.pitch) / dTime;
	pid->d.yaw = pid_params.KD.yaw * (-rotation.yaw + pid->prev.yaw) / dTime;

	//sum pid
	torque->roll = pid->p.roll + pid->i.roll + pid->d.roll;
	if (torque->roll > pid_params.max.roll)torque->roll = pid_params.max.roll;
	if (torque->roll < pid_params.min.roll)torque->roll = pid_params.min.roll;
	torque->pitch = pid->p.pitch + pid->i.pitch + pid->d.pitch;
	if (torque->pitch > pid_params.max.pitch)torque->pitch = pid_params.max.pitch;
	if (torque->pitch < pid_params.min.pitch)torque->pitch = pid_params.min.pitch;
	torque->yaw = pid->p.yaw + pid->i.yaw + pid->d.yaw;
	if (torque->yaw > pid_params.max.yaw)torque->yaw = pid_params.max.yaw;
	if (torque->yaw < pid_params.min.yaw)torque->yaw = pid_params.min.yaw;

	//prev values
	pid->prevE.yaw = pid->E.yaw;
	pid->prevE.pitch = pid->E.pitch;
	pid->prevE.roll = pid->E.roll;
	pid->prev.yaw = rotation.yaw;
	pid->prev.pitch = rotation.pitch;
	pid->prev.roll = rotation.roll;
}
