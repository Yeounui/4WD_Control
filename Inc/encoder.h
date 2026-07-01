#ifndef ENCODER_H
#define ENCODER_H

#include "motor.h"

/* One rising Hall edge is one wheel revolution. Pulses faster than this floor
 * imply more than 3000 RPM and are treated as bounce/noise.
 */
#define ENCODER_DEBOUNCE_FLOOR_MS 20U

/* With one magnet per wheel, speeds below about 30 RPM are reported as stopped
 * to avoid latching a stale period after the wheel stops.
 */
#define ENCODER_STALL_TIMEOUT_MS 2000U

void Encoder_Init(void);
void Encoder_OnPulse(MotorId wheel);
void Encoder_Sample(float dt);
float Encoder_GetSpeed(MotorId wheel);
uint32_t Encoder_GetCount(MotorId wheel);

#endif /* ENCODER_H */
