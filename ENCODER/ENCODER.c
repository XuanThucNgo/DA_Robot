/*
 * ENCODER.c
 *
 *  Created on: Dec 1, 2024
 *      Author: ADMIN
 */

#include "ENCODER.h"
#include "math.h"

void Encoder_Init(ENCODER_TypeDef* ENCODER, TIM_HandleTypeDef* htim, uint16_t pulsePerRev, float deltaT)
{
  ENCODER->htim = htim;
  ENCODER->encoder_PPR = pulsePerRev;
  ENCODER->deltaT = deltaT;
}

void Encoder_GetPulse(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode)
{
  ENCODER->count_timer += (int16_t)__HAL_TIM_GET_COUNTER(ENCODER->htim);
  __HAL_TIM_SET_COUNTER(ENCODER->htim, 0);

  switch (CountMode)
  {
  case MODE_x1:
    ENCODER->encoder_count_x1 = ENCODER->count_timer / 4;
    break;
  
  case MODE_x4:
    ENCODER->encoder_count_x4 = ENCODER->count_timer;
    break;
  }
}

float Encoder_GetPulse_return(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode)
{
  ENCODER->count_timer += (int16_t)__HAL_TIM_GET_COUNTER(ENCODER->htim);
  __HAL_TIM_SET_COUNTER(ENCODER->htim, 0);

  switch (CountMode)
  {
  case MODE_x1:
    ENCODER->encoder_count_x4 = ENCODER->count_timer / 4;
    break;
  
  case MODE_x4:
    ENCODER->encoder_count_x4 = ENCODER->count_timer;
    break;
  }

  return ENCODER->encoder_count_x4;  
}

float Encoder_GetSpeed(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode)
{
  switch (CountMode)
  {
  case MODE_x4:
    ENCODER->vel_Real = (((ENCODER->encoder_count_x4 - ENCODER->encoder_count_x4_pre) / ENCODER->deltaT) / ENCODER->encoder_PPR) * (2 * M_PI) * 0.0325; // rad/s * R_banh
    ENCODER->vel_Fil = 0.854 * ENCODER->vel_Fil + 0.0728 * ENCODER->vel_Real + 0.0728 * ENCODER->vel_Pre;

    ENCODER->vel_Pre = ENCODER->vel_Real;
		ENCODER->encoder_count_x4_pre = ENCODER->encoder_count_x4;

		return ENCODER->vel_Fil;
    break;
  
  case MODE_x1:
    ENCODER->vel_Real = (((ENCODER->encoder_count_x1 - ENCODER->encoder_count_x1_pre) / ENCODER->deltaT) / ENCODER->encoder_PPR) * (2 * M_PI) * 0.0325; // rad/s * R_banh
    ENCODER->vel_Fil = 0.854 * ENCODER->vel_Fil + 0.0728 * ENCODER->vel_Real + 0.0728 * ENCODER->vel_Pre;

    ENCODER->vel_Pre = ENCODER->vel_Real;
		ENCODER->encoder_count_x1_pre = ENCODER->encoder_count_x1;

		return ENCODER->vel_Fil;
    break;
  }

  return 0;
}
float Encoder_GetRound(ENCODER_TypeDef* ENCODER, ENCODER_MODE_t CountMode)
{
  switch (CountMode)
  {
  case MODE_x4:
    return ENCODER->currentPos = ((float)ENCODER->encoder_count_x4 / (float)ENCODER->encoder_PPR);
    break;
  
  case MODE_x1:
    return ENCODER->currentPos = ((float)ENCODER->encoder_count_x1 / (float)ENCODER->encoder_PPR);
    break;
  }

  return 0;
}
void Encoder_ResetCount(ENCODER_TypeDef* ENCODER)
{
  __HAL_TIM_SET_COUNTER(ENCODER->htim, 0);
	ENCODER->encoder_count_x4 = 0;
	ENCODER->encoder_count_x1 = 0;
	ENCODER->vel_Real = 0;
	ENCODER->vel_Pre = 0;
}

float Encoder_GetVelFil(ENCODER_TypeDef* ENCODER)
{
	return ENCODER->vel_Fil;
}




