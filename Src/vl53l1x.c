#include "vl53l1x.h"

#include "main.h"
#include "vl53l1_api.h"

#include <string.h>

#define VL53L1X_DEFAULT_ADDR             0x52U
#define VL53L1X_FRONT_ADDR               0x54U
#define VL53L1X_LEFT_ADDR                0x56U
#define VL53L1X_RIGHT_ADDR               0x58U

#define TOF_OBSTACLE_FRONT_MM            250U
#define TOF_OBSTACLE_SIDE_MM             180U
#define TOF_TIMING_BUDGET_US             50000U
#define TOF_INTER_MEASUREMENT_MS         70U

static const uint8_t tof_addr[TOF_COUNT] = {
  VL53L1X_FRONT_ADDR,
  VL53L1X_LEFT_ADDR,
  VL53L1X_RIGHT_ADDR
};

static VL53L1_Dev_t tof_dev[TOF_COUNT];
static TOF_Sample tof_sample[TOF_COUNT];
static uint8_t tof_initialized;

static void xshut_write(TOF_Sensor sensor, uint8_t state)
{
  switch (sensor)
  {
    case TOF_FRONT:
      if (state != 0U)
      {
        LL_GPIO_SetOutputPin(VL53_XSHUT_FRONT_GPIO_Port, VL53_XSHUT_FRONT_Pin);
      }
      else
      {
        LL_GPIO_ResetOutputPin(VL53_XSHUT_FRONT_GPIO_Port, VL53_XSHUT_FRONT_Pin);
      }
      break;

    case TOF_LEFT:
      if (state != 0U)
      {
        LL_GPIO_SetOutputPin(VL53_XSHUT_LEFT_GPIO_Port, VL53_XSHUT_LEFT_Pin);
      }
      else
      {
        LL_GPIO_ResetOutputPin(VL53_XSHUT_LEFT_GPIO_Port, VL53_XSHUT_LEFT_Pin);
      }
      break;

    case TOF_RIGHT:
      if (state != 0U)
      {
        LL_GPIO_SetOutputPin(VL53_XSHUT_RIGHT_GPIO_Port, VL53_XSHUT_RIGHT_Pin);
      }
      else
      {
        LL_GPIO_ResetOutputPin(VL53_XSHUT_RIGHT_GPIO_Port, VL53_XSHUT_RIGHT_Pin);
      }
      break;

    default:
      break;
  }
}

static uint8_t tof_configure_sensor(TOF_Sensor sensor)
{
  VL53L1_DEV dev;
  VL53L1_Error status;

  dev = &tof_dev[sensor];
  memset(dev, 0, sizeof(*dev));
  dev->i2c_slave_address = VL53L1X_DEFAULT_ADDR;

  xshut_write(sensor, 1U);
  HAL_Delay(5U);

  status = VL53L1_WaitDeviceBooted(dev);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }

  status = VL53L1_DataInit(dev);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }

  status = VL53L1_SetDeviceAddress(dev, tof_addr[sensor]);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }
  dev->i2c_slave_address = tof_addr[sensor];

  status = VL53L1_StaticInit(dev);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }
  status = VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, TOF_TIMING_BUDGET_US);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, TOF_INTER_MEASUREMENT_MS);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }
  status = VL53L1_StartMeasurement(dev);
  if (status != VL53L1_ERROR_NONE)
  {
    return 1U;
  }

  tof_sample[sensor].present = 1U;
  return 0U;
}

uint8_t TOF_Init_All(void)
{
  TOF_Sensor sensor;
  uint8_t ok;

  tof_initialized = 0U;
  ok = 1U;
  for (sensor = TOF_FRONT; sensor < TOF_COUNT; sensor++)
  {
    tof_sample[sensor].distance_mm = 0U;
    tof_sample[sensor].raw_distance_mm = 0U;
    tof_sample[sensor].range_status = 0U;
    tof_sample[sensor].ready = 0U;
    tof_sample[sensor].obstacle = 0U;
    tof_sample[sensor].present = 0U;
    xshut_write(sensor, 0U);
  }

  HAL_Delay(10U);
  for (sensor = TOF_FRONT; sensor < TOF_COUNT; sensor++)
  {
    if (tof_configure_sensor(sensor) != 0U)
    {
      xshut_write(sensor, 0U);
      ok = 0U;
    }
  }

  tof_initialized = ok;
  return (ok != 0U) ? 0U : 1U;
}

uint8_t TOF_ReadDistance_mm(TOF_Sensor sensor, uint16_t *distance_mm)
{
  VL53L1_RangingMeasurementData_t data;
  uint8_t ready;

  if ((sensor >= TOF_COUNT) || (distance_mm == (uint16_t *)0)
      || (tof_sample[sensor].present == 0U))
  {
    return 1U;
  }

  if (VL53L1_GetMeasurementDataReady(&tof_dev[sensor], &ready) != VL53L1_ERROR_NONE)
  {
    tof_sample[sensor].range_status = 0xFEU;
    tof_sample[sensor].ready = 0U;
    return 1U;
  }
  if (ready == 0U)
  {
    tof_sample[sensor].range_status = 0xFDU;
    tof_sample[sensor].ready = 0U;
    return 1U;
  }
  if (VL53L1_GetRangingMeasurementData(&tof_dev[sensor], &data) != VL53L1_ERROR_NONE)
  {
    tof_sample[sensor].range_status = 0xFCU;
    tof_sample[sensor].ready = 0U;
    return 1U;
  }
  (void)VL53L1_ClearInterruptAndStartMeasurement(&tof_dev[sensor]);

  tof_sample[sensor].raw_distance_mm = (data.RangeMilliMeter > 0) ? (uint16_t)data.RangeMilliMeter : 0U;
  tof_sample[sensor].range_status = data.RangeStatus;
  if ((data.RangeStatus != 0U) || (data.RangeMilliMeter <= 0))
  {
    tof_sample[sensor].ready = 0U;
    tof_sample[sensor].obstacle = 0U;
    return 1U;
  }

  *distance_mm = (uint16_t)data.RangeMilliMeter;
  tof_sample[sensor].distance_mm = *distance_mm;
  tof_sample[sensor].ready = 1U;
  tof_sample[sensor].obstacle =
      (*distance_mm < ((sensor == TOF_FRONT) ? TOF_OBSTACLE_FRONT_MM : TOF_OBSTACLE_SIDE_MM));

  return 0U;
}

uint8_t TOF_UpdateAll(void)
{
  TOF_Sensor sensor;
  uint8_t any_ok;

  if (tof_initialized == 0U)
  {
    return 1U;
  }

  any_ok = 0U;
  for (sensor = TOF_FRONT; sensor < TOF_COUNT; sensor++)
  {
    uint16_t distance;

    if (TOF_ReadDistance_mm(sensor, &distance) == 0U)
    {
      any_ok = 1U;
    }
  }

  return (any_ok != 0U) ? 0U : 1U;
}

uint8_t TOF_IsObstacle(void)
{
  TOF_Sensor sensor;

  for (sensor = TOF_FRONT; sensor < TOF_COUNT; sensor++)
  {
    if ((tof_sample[sensor].present != 0U) && (tof_sample[sensor].obstacle != 0U))
    {
      return 1U;
    }
  }

  return 0U;
}

TOF_Sample TOF_GetSample(TOF_Sensor sensor)
{
  TOF_Sample empty = {0U, 0U, 0U, 0U, 0U, 0U};

  if (sensor >= TOF_COUNT)
  {
    return empty;
  }

  return tof_sample[sensor];
}
