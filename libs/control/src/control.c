#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "stm32g4xx_hal.h"
#include "tim.h"

#include "control.h"

typedef struct
{
    TIM_HandleTypeDef *p_tim;
    uint32_t ch_r;
    uint32_t ch_l;

} control_config_t;

typedef struct
{
    float theta;
    float dtheta;
    float phi;
    float dphi;
    float psi;
    float dpsi;
} control_desc_t;

typedef struct
{
    float err;
    float err_i;
    float err_d;
    float last;
} error_t;

static control_config_t m_control_config;
static control_desc_t m_control_desc;
static error_t err_pitch, err_roll;

control_pid_t pid_pitch;
control_pid_t pid_roll;

void control_init(TIM_HandleTypeDef *p_pwm, uint32_t channel_right, uint32_t channel_left)
{
    m_control_config.p_tim = p_pwm;
    m_control_config.ch_r = channel_right;
    m_control_config.ch_l = channel_left;

    HAL_TIM_PWM_Start(p_pwm, channel_right);
    HAL_TIM_PWM_Start(p_pwm, channel_left);
}

void control_state_set(float theta, float dtheta, float phi, float dphi, float psi, float dpsi)
{
    m_control_desc.theta = theta;
    m_control_desc.dtheta = dtheta;
    m_control_desc.phi = phi;
    m_control_desc.dphi = dphi;
    m_control_desc.psi = psi;
    m_control_desc.dpsi = dpsi;
}

void control_pid_set(control_pid_t *p_pid, float p, float i, float d)
{
    p_pid->p = p;
    p_pid->i = i;
    p_pid->d = d;
}

void control_errors_clear()
{
    err_pitch.err = 0;
    err_pitch.err_i = 0;
    err_pitch.err_d = 0;
    err_pitch.last = 0;

    err_roll.err = 0;
    err_roll.err_i = 0;
    err_roll.err_d = 0;
}

void control_signal_get(int8_t *control_r, int8_t *control_l, control_state_t *p_state)
{
    int16_t control;
    err_pitch.err = m_control_desc.theta + p_state->pitch;
    err_pitch.err_i += err_pitch.err * 0.01f;
    err_pitch.err_d = -(p_state->pitch - err_pitch.last) / 0.01f;
    err_pitch.last = p_state->pitch;

    if (err_pitch.err_i > 400)
        err_pitch.err_i = 400;
    else if (err_pitch.err_i < -400)
        err_pitch.err_i = -400;

    err_roll.err = m_control_desc.phi + p_state->roll;
    err_roll.err_i += err_roll.err * 0.01f;
    err_roll.err_d = p_state->droll;
    if (err_roll.err_i > 400)
        err_roll.err_i = 400;
    else if (err_roll.err_i < -400)
        err_roll.err_i = -400;

    control = (int16_t)(err_pitch.err * pid_pitch.p +
                        err_pitch.err_i * pid_pitch.i +
                        err_pitch.err_d * pid_pitch.d +
                        err_roll.err * pid_roll.p +
                        err_roll.err_i * pid_roll.i +
                        err_roll.err_d * pid_roll.d);

    if (control > 100)
    {
        control = 100;
    }
    else if (control < -100)
    {
        control = -100;
    }

    *control_l = (int8_t)control;
    *control_r = (int8_t)control;
}

void control_dirve_motors(int8_t control, control_channel_t ch)
{
    uint32_t pwm = (uint32_t)abs((int)((control / 100.f) * MAX_CONTROL_PWM));

    GPIO_TypeDef *gpio1 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction1_GPIO_Port : Motor_L_direction1_GPIO_Port;
    GPIO_TypeDef *gpio2 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction2_GPIO_Port : Motor_L_direction2_GPIO_Port;

    uint16_t pin1 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction1_Pin : Motor_L_direction1_Pin;
    uint16_t pin2 = ch == CONTROL_RIGHT_WHEEL ? Motor_R_direction2_Pin : Motor_L_direction2_Pin;

    pwm = pwm > MAX_CONTROL_PWM ? MAX_CONTROL_PWM : pwm;

    if (control > 0)
    {
        HAL_GPIO_WritePin(gpio1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(gpio2, pin2, GPIO_PIN_SET);
    }
    else if (control < 0)
    {
        HAL_GPIO_WritePin(gpio1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(gpio2, pin2, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(gpio1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(gpio2, pin2, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(m_control_config.p_tim, ch, pwm);
}
