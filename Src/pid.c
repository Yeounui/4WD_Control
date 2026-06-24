#include "pid.h"

static float PID_Clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

void PID_Init(PID *p, float kp, float ki, float kd, float out_min, float out_max)
{
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
  p->integral = 0.0f;
  p->prev_error = 0.0f;
  p->out_min = out_min;
  p->out_max = out_max;
  p->i_min = out_min;
  p->i_max = out_max;
}

void PID_Reset(PID *p)
{
  p->integral = 0.0f;
  p->prev_error = 0.0f;
}

float PID_Update(PID *p, float setpoint, float measured, float dt)
{
  float error;
  float derivative;
  float output;

  error = setpoint - measured;

  if (dt > 0.0f)
  {
    p->integral += error * dt;
    p->integral = PID_Clamp(p->integral, p->i_min, p->i_max);
    derivative = (error - p->prev_error) / dt;
  }
  else
  {
    derivative = 0.0f;
  }

  output = (p->kp * error) + (p->ki * p->integral) + (p->kd * derivative);
  output = PID_Clamp(output, p->out_min, p->out_max);

  if (dt > 0.0f)
  {
    p->prev_error = error;
  }

  return output;
}
