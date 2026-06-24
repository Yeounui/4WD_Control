#ifndef PARAMSTORE_H
#define PARAMSTORE_H

#include <stdint.h>

#define PARAMSTORE_ADDRESS  0x0801FC00U
#define PARAMSTORE_MAGIC    0x4B414C31U

uint8_t ParamStore_Load(float *r, float *bias);
uint8_t ParamStore_Save(float r, float bias);

#endif /* PARAMSTORE_H */
