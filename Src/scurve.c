#include "scurve.h"

#include <math.h>
#include <stddef.h>

void SCurve_Init(SCurve *sc, float target, float accel_max, float jerk)
{
  if (sc == NULL)
  {
    return;
  }

  sc->current = 0.0f;
  sc->accel = 0.0f;
  sc->target = target;
  sc->accel_max = fabsf(accel_max);
  sc->jerk = fabsf(jerk);
  sc->phase = SCURVE_PHASE_ACCEL;
}

float SCurve_Update(SCurve *sc, float dt)
{
  if (sc == NULL)
  {
    return 0.0f;
  }

  if (dt <= 0.0f)
  {
    return sc->current;
  }

  if (sc->jerk <= 0.0f)
  {
    sc->current = sc->target;
    sc->accel = 0.0f;
    if (sc->phase == SCURVE_PHASE_DECEL)
    {
      sc->phase = SCURVE_PHASE_DONE;
    }
    else
    {
      sc->phase = SCURVE_PHASE_CRUISE;
    }
    return sc->current;
  }

  switch (sc->phase)
  {
  case SCURVE_PHASE_ACCEL:
  {
    float gap = sc->target - sc->current;
    float stop_gap = (sc->accel * sc->accel) / (2.0f * sc->jerk);

    if (gap <= stop_gap)
    {
      sc->accel -= sc->jerk * dt;
      if (sc->accel < 0.0f)
      {
        sc->accel = 0.0f;
      }
    }
    else if (sc->accel < sc->accel_max)
    {
      sc->accel += sc->jerk * dt;
      if (sc->accel > sc->accel_max)
      {
        sc->accel = sc->accel_max;
      }
    }
    else
    {
      sc->accel = sc->accel_max;
    }

    sc->current += sc->accel * dt;
    if (sc->current >= sc->target)
    {
      sc->current = sc->target;
      sc->accel = 0.0f;
      sc->phase = SCURVE_PHASE_CRUISE;
    }
    break;
  }

  case SCURVE_PHASE_CRUISE:
    if (sc->target < sc->current)
    {
      sc->phase = SCURVE_PHASE_DECEL;
    }
    break;

  case SCURVE_PHASE_DECEL:
  {
    float gap = sc->current - sc->target;
    float stop_gap = (sc->accel * sc->accel) / (2.0f * sc->jerk);

    if (gap <= stop_gap)
    {
      sc->accel -= sc->jerk * dt;
      if (sc->accel < 0.0f)
      {
        sc->accel = 0.0f;
      }
    }
    else if (sc->accel < sc->accel_max)
    {
      sc->accel += sc->jerk * dt;
      if (sc->accel > sc->accel_max)
      {
        sc->accel = sc->accel_max;
      }
    }
    else
    {
      sc->accel = sc->accel_max;
    }

    sc->current -= sc->accel * dt;
    if (sc->current <= sc->target)
    {
      sc->current = sc->target;
      sc->accel = 0.0f;
      sc->phase = SCURVE_PHASE_DONE;
    }
    break;
  }

  case SCURVE_PHASE_DONE:
  default:
    sc->current = sc->target;
    sc->accel = 0.0f;
    break;
  }

  return sc->current;
}
