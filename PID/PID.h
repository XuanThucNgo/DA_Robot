/*
 * PID.h
 *
 *  Created on: Dec 1, 2024
 *      Author: ADMIN
 */

#ifndef PID_H_
#define PID_H_

// PID Paramters
typedef struct PID
{
  float err;
  float err_pre;

  float Kp;
  float Ki;
  float Kd;
  float Kb;

  float sample_time;
  float alpha;

	float PTerm;
	float ITerm;
	float DTerm;
	float DTermFil;
	float DTermFilPre;

	float uSat;
	float u;
	float uAboveLimit;
	float uUnderLimit;

} PID_TypeDef;

void PID_Init(PID_TypeDef* pid, float Kp, float Ki, float Kd, float alpha, float sample_time, float uAboveLimit, float uUnderLimit);
void PID_clear(PID_TypeDef* pid);
float PID(PID_TypeDef* pid, float Target, float Current);
void PID_reset(PID_TypeDef* pid);

#endif /* PID_H_ */
