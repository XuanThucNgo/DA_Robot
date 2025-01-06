/*
 * ENCODER.h
 *
 *  Created on: Dec 1, 2024
 *      Author: ADMIN
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"

typedef struct ENCODER
{
	//Timer and Count Parameters
	TIM_HandleTypeDef* htim;
	int32_t count_timer;

	int32_t encoder_count_x4;
	int32_t encoder_count_x4_pre;

	int32_t encoder_count_x1;
	int32_t encoder_count_x1_pre;

	int32_t encoder_PPR;

	//Speed Parameters
	float vel_Real;
	float vel_Pre;
	float vel_Fil;

	float deltaT;

	//POS
	float currentPos;
} ENCODER_TypeDef;

typedef enum ENCODER_MODE
{
	MODE_x1,
	MODE_x4,
}	ENCODER_MODE_t;

void Encoder_Init(ENCODER_TypeDef* ENCODER, TIM_HandleTypeDef* htim, uint16_t pulsePerRev, float deltaT);
void Encoder_GetPulse(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode);
float Encoder_GetSpeed(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode);
float Encoder_GetRound(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode);
void Encoder_ResetCount(ENCODER_TypeDef* ENCODER);
float Encoder_GetVelFil(ENCODER_TypeDef* ENCODER);
float Encoder_GetPulse_return(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode);


#endif /* ENCODER_H_ */
