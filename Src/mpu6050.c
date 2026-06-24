#include "mpu6050.h"

#include "main.h"

#include <math.h>

#define MPU6050_ADDRESS        0xD0U
#define MPU6050_WHO_AM_I       0x75U
#define MPU6050_PWR_MGMT_1     0x6BU
#define MPU6050_GYRO_CONFIG    0x1BU
#define MPU6050_ACCEL_CONFIG   0x1CU
#define MPU6050_CONFIG         0x1AU
#define MPU6050_ACCEL_XOUT_H   0x3BU

extern I2C_HandleTypeDef hi2c1;

uint8_t MPU6050_Init(void)
{
  uint8_t value;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &value, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    return 1U;
  }

  if (value != 0x68U)
  {
    return 1U;
  }

  value = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    return 1U;
  }

  value = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    return 1U;
  }

  value = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    return 1U;
  }

  value = 0x03U;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    return 1U;
  }

  return 0U;
}

uint8_t MPU6050_Read(float *omega_dps, float *accel_angle)
{
  uint8_t data[14];
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_z;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, data, sizeof(data), HAL_MAX_DELAY) != HAL_OK)
  {
    return 1U;
  }

  accel_y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  accel_z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  gyro_z = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

  *omega_dps = (float)gyro_z / 131.0f;

  /* atan2(Y, Z) is a gravity-referenced tilt angle, not true yaw. */
  *accel_angle = atan2f((float)accel_y, (float)accel_z) * 57.2957795f;

  return 0U;
}
