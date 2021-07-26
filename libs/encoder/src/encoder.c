#include "tim.h"

#include "encoder.h"

volatile encoder_t encoder_right, encoder_left;

static inline void overflow_underflow_handler(encoder_t*);

void encoder_init(TIM_HandleTypeDef *left_tim, TIM_HandleTypeDef *right_tim, uint8_t wheel_diameter_mm) {
    encoder_left.tim = left_tim;
    encoder_right.tim = right_tim;

    encoder_left.wheel_r = wheel_diameter_mm;
    encoder_right.wheel_r = wheel_diameter_mm;

    HAL_TIM_Encoder_Start(encoder_left.tim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(encoder_right.tim, TIM_CHANNEL_ALL);
}

void encoder_irq_handler(uint16_t delta_ms) {
    encoder_left.encoder_cnt_new = (uint16_t)__HAL_TIM_GetCounter(encoder_left.tim);
    encoder_right.encoder_cnt_new = (uint16_t)__HAL_TIM_GetCounter(encoder_right.tim);

    int16_t encoder_delta_l, encoder_delta_r;
    encoder_delta_l = (int16_t)(encoder_left.encoder_cnt_new - encoder_left.encoder_cnt_old);
    encoder_delta_r = (int16_t)(encoder_right.encoder_cnt_new - encoder_right.encoder_cnt_old);
    encoder_left.encoder_cnt_old = encoder_left.encoder_cnt_new;
    encoder_right.encoder_cnt_old = encoder_right.encoder_cnt_new;

    float velocity_l, velocity_r;
    velocity_l = (float)((encoder_delta_l*60000)/IMPUSLES_PER_TURN)/delta_ms;
    velocity_r = (float)((encoder_delta_r*60000)/IMPUSLES_PER_TURN) / delta_ms;


    encoder_left.ang_velo = velocity_l;
    encoder_right.ang_velo = velocity_r;
}

float encoder_get_angle_deg(encoder_t *hencoder) {
    float degree;
    uint32_t counts = hencoder->tim->Instance->CNT;
    if (hencoder->full_turn < 0) {
        degree = (float)hencoder->full_turn * 360.0F + ((float)counts * 360.0F) / IMPUSLES_PER_TURN;
    }
    else {
        degree = (float)(hencoder->full_turn - 1) * 360.0F + ((float)counts * 360.0F) / IMPUSLES_PER_TURN;
    }
    return degree;
}

void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

void TIM4_IRQHandler(void) {
    encoder_irq_handler(1);
    HAL_TIM_IRQHandler(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        overflow_underflow_handler((encoder_t*)&encoder_left);
    } else if (htim->Instance == TIM2) {
        overflow_underflow_handler((encoder_t*)&encoder_right);
    }
}

static inline void overflow_underflow_handler(encoder_t *hencoder) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(hencoder->tim) > 0){
        hencoder->full_turn--;
    } else {
        hencoder->full_turn++;
    }
}

