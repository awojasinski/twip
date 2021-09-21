#ifndef CONTROL_H
#define CONTROL_H

#include "tim.h"
#include "gpio.h"

#define MAX_CONTROL_PWM 1062

typedef enum controler_type_t
{
    CONTROLER_P,
    CONTROLER_PI,
    CONTROLER_PD,
    CONTROLER_PID,
    CONTROLER_LQ
} controler_type_t;

typedef enum control_channel_t
{
    CONTROL_RIGHT_WHEEL = TIM_CHANNEL_4,
    CONTROL_LEFT_WHEEL = TIM_CHANNEL_1,
} control_channel_t;

typedef struct
{
    float pitch;
    float dpitch;
    float yaw;
    float dyaw;
    float roll;
    float droll;

} control_state_t;

typedef struct
{
    float p;
    float i;
    float d;
} control_pid_t;

extern control_pid_t pid_pitch;
extern control_pid_t pid_roll;

void control_init(TIM_HandleTypeDef *, uint32_t, uint32_t);
void control_algorithm_set(controler_type_t);
void control_state_set(float, float, float, float, float, float);
void control_pid_set(control_pid_t *, float, float, float);
void control_signal_get(int8_t *, int8_t *, control_state_t *);
void control_errors_clear(void);
void control_dirve_motors(int8_t, control_channel_t);

#endif
