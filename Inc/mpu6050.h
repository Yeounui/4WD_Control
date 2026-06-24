#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

uint8_t MPU6050_Init(void);
uint8_t MPU6050_Read(float *omega_dps, float *accel_angle);

#endif /* MPU6050_H */
