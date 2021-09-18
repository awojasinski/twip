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

static control_config_t m_control_config;
static control_desc_t m_control_desc;

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

void control_signal_get(int8_t *control_r, int8_t *control_l, control_state_t *p_state)
{
    static float error_pitch, error_pitch_i;
    static float error_roll, error_roll_i;

    int16_t control;
    error_pitch = m_control_desc.theta - p_state->pitch;
    error_pitch_i += error_pitch * 0.01f;
    if (error_pitch_i > 400)
        error_pitch_i = 400;
    else if (error_pitch_i < -400)
        error_pitch_i = -400;

    error_roll = m_control_desc.dphi - p_state->droll;
    error_roll_i += error_roll * 0.01f;
    if (error_roll_i > 400)
        error_roll_i = 400;
    else if (error_roll_i < -400)
        error_roll_i = -400;

    control = (int16_t)(error_pitch * pid_pitch.p +
                        error_pitch_i * pid_pitch.i +
                        error_roll * pid_roll.p +
                        error_roll_i * pid_roll.i);

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
    uint32_t pwm = (uint32_t)abs((int)((control / 100) * (int)MAX_CONTROL_PWM));

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
