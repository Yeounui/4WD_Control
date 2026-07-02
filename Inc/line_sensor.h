#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f1xx_hal.h"

#define LINE_SENSOR_ADC_MAX_RAW 4095U

typedef struct
{
  uint16_t raw_min;
  uint16_t raw_max;
  float center;
  uint8_t invert;
} LineSensorConfig;

typedef struct
{
  uint16_t raw;
  float normalized;
  float error;
  uint8_t fresh;
  uint8_t active;
} LineSensorSample;

extern const LineSensorConfig LINE_SENSOR_DEFAULT_CONFIG;

HAL_StatusTypeDef LineSensor_Init(const LineSensorConfig *config);
void LineSensor_Stop(void);
uint8_t LineSensor_IsActive(void);
uint8_t LineSensor_HasFreshSample(void);
void LineSensor_ClearFresh(void);
LineSensorSample LineSensor_Read(void);
uint16_t LineSensor_GetRaw(void);
float LineSensor_GetNormalized(void);
float LineSensor_GetCentroid(void);
float LineSensor_GetError(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_SENSOR_H */
