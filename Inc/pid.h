#ifndef PID_H
#define PID_H

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float out_min;
  float out_max;
  float i_min;
  float i_max;
} PID;

void PID_Init(PID *p, float kp, float ki, float kd, float out_min, float out_max);
void PID_Reset(PID *p);
float PID_Update(PID *p, float setpoint, float measured, float dt);

#endif /* PID_H */
