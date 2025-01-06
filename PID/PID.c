/*
 * PID.c
 *
 *  Created on: Dec 1, 2024
 *      Author: ADMIN
 */

#include "PID.h"

void PID_Init(PID_TypeDef* pid, float Kp, float Ki, float Kd, float alpha, float sample_time, float uAboveLimit, float uUnderLimit)
{
  PID_clear(pid);
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->alpha = alpha;
  pid->sample_time = sample_time;
  pid->Kb = 0;
  pid->uAboveLimit = uAboveLimit;
  pid->uUnderLimit = uUnderLimit;
}

void PID_clear(PID_TypeDef* pid)
{
	pid->PTerm = 0;
	pid->ITerm = 0;
	pid->DTerm = 0;
	pid->DTermFil = 0;
	pid->DTermFilPre = 0;
	pid->u = 0;
	pid->uSat = 0;
}

float PID(PID_TypeDef* pid, float Target, float Current)
{
	pid->err = Target - Current;

	if(!pid->Ki) pid->Kb = 0.0;
	else pid->Kb = 1/(pid->sample_time);

	// P Term
	pid->PTerm = pid->Kp * pid->err;

	// D Term
	pid->DTerm = pid->Kd * (pid->err - pid->err_pre) / (pid->sample_time);
	pid->DTermFil = pid->alpha * pid->DTerm + (1 - pid->alpha) * pid->DTermFilPre;

	// I Term
	pid->ITerm += (pid->Ki * pid->err + pid->Kb * (pid->uSat - pid->u)) * (pid->sample_time);

	pid->err_pre = pid->err;
	pid->DTermFilPre = pid->DTermFil;

	// PID
	pid->u = pid->PTerm + pid->ITerm + pid->DTermFil;

	// Saturation of PID
	if (pid->u > pid->uAboveLimit)	pid->uSat = pid->uAboveLimit;
	else if (pid->u < pid->uUnderLimit) pid->uSat = pid->uUnderLimit;
	else pid->uSat = pid->u;

	return pid->uSat;
}

void PID_reset(PID_TypeDef* pid)
{
	pid->u = 0;
	pid->uSat = 0;
}


