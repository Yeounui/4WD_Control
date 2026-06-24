#ifndef KALMAN_H
#define KALMAN_H

void Kalman_Init(void);
float Kalman_Update(float accel_angle, float omega_dps, float dt);
void Kalman_SetR(float r);
void Kalman_SetBias(float bias);
float Kalman_GetR(void);
float Kalman_GetBias(void);
float Kalman_GetAngle(void);

#endif /* KALMAN_H */
