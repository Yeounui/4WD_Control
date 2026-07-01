#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdint.h>

typedef enum
{
  TOF_FRONT = 0,
  TOF_LEFT,
  TOF_RIGHT,
  TOF_COUNT
} TOF_Sensor;

typedef struct
{
  uint16_t distance_mm;
  uint16_t raw_distance_mm;
  uint8_t range_status;
  uint8_t ready;
  uint8_t obstacle;
  uint8_t present;
} TOF_Sample;

uint8_t TOF_Init_All(void);
uint8_t TOF_UpdateAll(void);
uint8_t TOF_ReadDistance_mm(TOF_Sensor sensor, uint16_t *distance_mm);
uint8_t TOF_IsObstacle(void);
TOF_Sample TOF_GetSample(TOF_Sensor sensor);

#endif /* VL53L1X_H */
