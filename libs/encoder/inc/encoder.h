#ifndef ENCODER_H
#define ENCODER_H

#include "stm32g4xx_hal.h"

//#define IMPUSLES_PER_TURN 2069
#define IMPUSLES_PER_TURN 2059

typedef struct
{
    TIM_HandleTypeDef *tim;
    int32_t encoder_cnt_old;
    int32_t encoder_cnt_new;
    int16_t full_turn;
    uint8_t wheel_r;
} encoder_t;

extern encoder_t encoder_right, encoder_left;

void encoder_init(TIM_HandleTypeDef *, TIM_HandleTypeDef *, uint8_t);
float encoder_get_angle_deg(encoder_t *);
float encoder_get_velo(encoder_t *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM7_IRQHandler(void);

#endif
