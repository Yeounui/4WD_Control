#include "line_sensor.h"

#define LINE_SENSOR_DMA_BUFFER_LEN 8U

extern ADC_HandleTypeDef hadc1;

const LineSensorConfig LINE_SENSOR_DEFAULT_CONFIG = {
  0U,
  LINE_SENSOR_ADC_MAX_RAW,
  0.5f,
  0U
};

static uint16_t line_sensor_dma_buffer[LINE_SENSOR_DMA_BUFFER_LEN];
static volatile uint8_t line_sensor_active;
static volatile uint8_t line_sensor_fresh;
static LineSensorConfig line_sensor_config = {
  0U,
  LINE_SENSOR_ADC_MAX_RAW,
  0.5f,
  0U
};

static float LineSensor_Clamp01(float value)
{
  if (value < 0.0f)
  {
    return 0.0f;
  }

  if (value > 1.0f)
  {
    return 1.0f;
  }

  return value;
}

static uint16_t LineSensor_AverageRaw(void)
{
  uint32_t sum = 0U;
  uint32_t i;

  for (i = 0U; i < LINE_SENSOR_DMA_BUFFER_LEN; i++)
  {
    sum += line_sensor_dma_buffer[i];
  }

  return (uint16_t)(sum / LINE_SENSOR_DMA_BUFFER_LEN);
}

static float LineSensor_NormalizeRaw(uint16_t raw)
{
  float normalized;
  uint16_t raw_min = line_sensor_config.raw_min;
  uint16_t raw_max = line_sensor_config.raw_max;

  if (raw_max <= raw_min)
  {
    return 0.0f;
  }

  normalized = ((float)raw - (float)raw_min) / ((float)raw_max - (float)raw_min);
  normalized = LineSensor_Clamp01(normalized);

  if (line_sensor_config.invert != 0U)
  {
    normalized = 1.0f - normalized;
  }

  return normalized;
}

static LineSensorSample LineSensor_MakeSample(uint8_t clear_fresh)
{
  LineSensorSample sample;

  sample.active = line_sensor_active;
  sample.fresh = line_sensor_fresh;

  if (line_sensor_active == 0U)
  {
    sample.raw = 0U;
    sample.normalized = line_sensor_config.center;
    sample.error = 0.0f;
    sample.fresh = 0U;
    return sample;
  }

  sample.raw = LineSensor_AverageRaw();
  sample.normalized = LineSensor_NormalizeRaw(sample.raw);
  sample.error = sample.normalized - line_sensor_config.center;

  if (clear_fresh != 0U)
  {
    line_sensor_fresh = 0U;
  }

  return sample;
}

HAL_StatusTypeDef LineSensor_Init(const LineSensorConfig *config)
{
  HAL_StatusTypeDef status;

  if (config != (const LineSensorConfig *)0)
  {
    line_sensor_config = *config;
    line_sensor_config.center = LineSensor_Clamp01(line_sensor_config.center);
  }
  else
  {
    line_sensor_config = LINE_SENSOR_DEFAULT_CONFIG;
  }

  if (line_sensor_active != 0U)
  {
    (void)HAL_ADC_Stop_DMA(&hadc1);
  }

  line_sensor_active = 0U;
  line_sensor_fresh = 0U;

  status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)line_sensor_dma_buffer,
                             LINE_SENSOR_DMA_BUFFER_LEN);
  if (status == HAL_OK)
  {
    line_sensor_active = 1U;
  }

  return status;
}

void LineSensor_Stop(void)
{
  if (line_sensor_active != 0U)
  {
    (void)HAL_ADC_Stop_DMA(&hadc1);
  }

  line_sensor_active = 0U;
  line_sensor_fresh = 0U;
}

uint8_t LineSensor_IsActive(void)
{
  return line_sensor_active;
}

uint8_t LineSensor_HasFreshSample(void)
{
  return line_sensor_fresh;
}

void LineSensor_ClearFresh(void)
{
  line_sensor_fresh = 0U;
}

LineSensorSample LineSensor_Read(void)
{
  return LineSensor_MakeSample(1U);
}

uint16_t LineSensor_GetRaw(void)
{
  return LineSensor_MakeSample(0U).raw;
}

float LineSensor_GetNormalized(void)
{
  return LineSensor_MakeSample(0U).normalized;
}

float LineSensor_GetCentroid(void)
{
  return LineSensor_GetNormalized();
}

float LineSensor_GetError(void)
{
  return LineSensor_MakeSample(0U).error;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    line_sensor_fresh = 1U;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    line_sensor_fresh = 1U;
  }
}
