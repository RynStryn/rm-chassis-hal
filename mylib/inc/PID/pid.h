#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx_hal.h"

enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};

typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;
  float output_deadband;

  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void (*f_param_init)(struct pid_t *pid,
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       float      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);

}pid_t;

void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    float intergral_limit,
    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float fdb, float ref);
void  pid_clear(pid_t *pid);
void abs_limit(float *a, float ABS_MAX);


#endif
