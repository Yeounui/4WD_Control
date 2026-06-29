#include "vl53l1_platform.h"

#include "main.h"
#include "soft_i2c.h"

#include <stddef.h>

VL53L1_Error VL53L1_CommsInitialise(
  VL53L1_Dev_t *pdev,
  uint8_t comms_type,
  uint16_t comms_speed_khz)
{
  if (pdev == NULL)
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }

  pdev->comms_type = comms_type;
  pdev->comms_speed_khz = comms_speed_khz;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_CommsClose(VL53L1_Dev_t *pdev)
{
  (void)pdev;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WriteMulti(
  VL53L1_Dev_t *pdev,
  uint16_t index,
  uint8_t *pdata,
  uint32_t count)
{
  if ((pdev == NULL) || ((pdata == NULL) && (count != 0U)) || (count > 0xFFFFU))
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }

  return (SoftI2C_MemWrite16(pdev->i2c_slave_address, index, pdata, (uint16_t)count) == 0U)
             ? VL53L1_ERROR_NONE
             : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_ReadMulti(
  VL53L1_Dev_t *pdev,
  uint16_t index,
  uint8_t *pdata,
  uint32_t count)
{
  if ((pdev == NULL) || ((pdata == NULL) && (count != 0U)) || (count > 0xFFFFU))
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }

  return (SoftI2C_MemRead16(pdev->i2c_slave_address, index, pdata, (uint16_t)count) == 0U)
             ? VL53L1_ERROR_NONE
             : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t data)
{
  return VL53L1_WriteMulti(pdev, index, &data, 1U);
}

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t data)
{
  uint8_t buf[2];

  buf[0] = (uint8_t)(data >> 8);
  buf[1] = (uint8_t)(data & 0xFFU);
  return VL53L1_WriteMulti(pdev, index, buf, sizeof(buf));
}

VL53L1_Error VL53L1_WrDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t data)
{
  uint8_t buf[4];

  buf[0] = (uint8_t)(data >> 24);
  buf[1] = (uint8_t)((data >> 16) & 0xFFU);
  buf[2] = (uint8_t)((data >> 8) & 0xFFU);
  buf[3] = (uint8_t)(data & 0xFFU);
  return VL53L1_WriteMulti(pdev, index, buf, sizeof(buf));
}

VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata)
{
  return VL53L1_ReadMulti(pdev, index, pdata, 1U);
}

VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t *pdata)
{
  uint8_t buf[2];
  VL53L1_Error status;

  if (pdata == NULL)
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }

  status = VL53L1_ReadMulti(pdev, index, buf, sizeof(buf));
  if (status == VL53L1_ERROR_NONE)
  {
    *pdata = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  }
  return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t *pdata)
{
  uint8_t buf[4];
  VL53L1_Error status;

  if (pdata == NULL)
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }

  status = VL53L1_ReadMulti(pdev, index, buf, sizeof(buf));
  if (status == VL53L1_ERROR_NONE)
  {
    *pdata = ((uint32_t)buf[0] << 24)
             | ((uint32_t)buf[1] << 16)
             | ((uint32_t)buf[2] << 8)
             | buf[3];
  }
  return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
  volatile int32_t i;

  (void)pdev;
  if (wait_us <= 0)
  {
    return VL53L1_ERROR_NONE;
  }

  for (i = 0; i < (wait_us * (int32_t)(SystemCoreClock / 4000000U)); i++)
  {
  }
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
  (void)pdev;
  if (wait_ms > 0)
  {
    HAL_Delay((uint32_t)wait_ms);
  }
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
  if (ptimer_freq_hz == NULL)
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }
  *ptimer_freq_hz = 1000;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count)
{
  if (ptimer_count == NULL)
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }
  *ptimer_count = (int32_t)HAL_GetTick();
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(VL53L1_Dev_t *pdev, uint32_t *ptime_ms)
{
  (void)pdev;
  if (ptime_ms == NULL)
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }
  *ptime_ms = HAL_GetTick();
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
  VL53L1_Dev_t *pdev,
  uint32_t timeout_ms,
  uint16_t index,
  uint8_t value,
  uint8_t mask,
  uint32_t poll_delay_ms)
{
  uint32_t start;
  uint8_t data;

  start = HAL_GetTick();
  do
  {
    if (VL53L1_RdByte(pdev, index, &data) != VL53L1_ERROR_NONE)
    {
      return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    if ((data & mask) == value)
    {
      return VL53L1_ERROR_NONE;
    }
    HAL_Delay(poll_delay_ms);
  } while ((HAL_GetTick() - start) < timeout_ms);

  return VL53L1_ERROR_TIME_OUT;
}

VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode)
{
  (void)pin;
  (void)mode;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioSetValue(uint8_t pin, uint8_t value)
{
  (void)pin;
  (void)value;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
  (void)pin;
  if (pvalue == NULL)
  {
    return VL53L1_ERROR_INVALID_PARAMS;
  }
  *pvalue = 0U;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioXshutdown(uint8_t value)
{
  (void)value;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioCommsSelect(uint8_t value)
{
  (void)value;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioPowerEnable(uint8_t value)
{
  (void)value;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
  (void)function;
  (void)edge_type;
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioInterruptDisable(void)
{
  return VL53L1_ERROR_NONE;
}
