#ifndef ENCODER_H
#define ENCODER_H

#include "stm32g4xx_hal.h"

//#define IMPUSLES_PER_TURN 2069
#define IMPUSLES_PER_TURN 1979

typedef struct
{
    TIM_HandleTypeDef *tim;
    uint16_t encoder_cnt_old;
    uint16_t encoder_cnt_new;
    int32_t full_turn;
    uint8_t wheel_r;
    float ang_velo;
    float ang_accel;
    uint32_t velo;
    float accel;
} encoder_t;

extern encoder_t encoder_right, encoder_left;

void encoder_init(TIM_HandleTypeDef *, TIM_HandleTypeDef *, uint8_t);
void encoder_irq_handler(uint16_t);
float encoder_get_angle_deg(encoder_t *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

#endif
