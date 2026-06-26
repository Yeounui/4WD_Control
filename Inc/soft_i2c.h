#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>

void SoftI2C_Init(void);
uint8_t SoftI2C_MemRead(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t SoftI2C_MemWrite(uint8_t dev_addr, uint8_t reg, const uint8_t *buf, uint16_t len);
uint8_t SoftI2C_MasterTransmit(uint8_t dev_addr, const uint8_t *buf, uint16_t len);
uint8_t SoftI2C_IsDeviceReady(uint8_t dev_addr);

#endif /* SOFT_I2C_H */
