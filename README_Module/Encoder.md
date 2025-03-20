Return [README.md](../README.md)

Reference [ENCODER.h](../ENCODER/ENCODER.h)

Reference [ENCODER.c](../ENCODER/ENCODER.c)

## Data structure for handling encoder measurements and calculations.
```
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
```
Add a requirement to implement the ENCODER_MODE operation mode for the encoder. In ENCODER.h, define an enum as follows:
```
typedef enum ENCODER_MODE
{
	MODE_x1,
	MODE_x4,
}	ENCODER_MODE_t;
```

# Functions API
### Encoder_Init
### Encoder_GetPulse
### Encoder_GetSpeed
### Encoder_ResetCount
### Encoder_GetVelFil
### Encoder_GetPulse_return


