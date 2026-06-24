#ifndef ENCODER_H
#define ENCODER_H

#include "motor.h"

/* PLACEHOLDER — set to the real per-wheel count per revolution
 * (Hall magnets x 2 because both edges are counted). USER must tune to hardware.
 */
#define ENCODER_COUNTS_PER_REV 20.0f

void Encoder_Init(void);
void Encoder_OnPulse(MotorId wheel);
void Encoder_Sample(float dt);
float Encoder_GetSpeed(MotorId wheel);

#endif /* ENCODER_H */
