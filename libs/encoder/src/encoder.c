#include "tim.h"

#include "encoder.h"

encoder_t encoder_right, encoder_left;

static inline void overflow_underflow_handler(encoder_t*);

void encoder_init(TIM_HandleTypeDef *left_tim, TIM_HandleTypeDef *right_tim, uint8_t wheel_diameter_mm) {
    MX_TIM2_Init(); // encoder right motor
    MX_TIM3_Init(); // encoder left motor

    encoder_left.tim = left_tim;
    encoder_right.tim = right_tim;

    encoder_left.full_turn = -1;
    encoder_right.full_turn = -1;

    encoder_left.wheel_r = wheel_diameter_mm;
    encoder_right.wheel_r = wheel_diameter_mm;

    HAL_TIM_Encoder_Start(encoder_left.tim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(encoder_right.tim, TIM_CHANNEL_ALL);

    HAL_TIM_Base_Start_IT(encoder_left.tim);
}

float encoder_get_angle_deg(encoder_t *hencoder) {
    float degree;
    uint32_t counts = hencoder->tim->Instance->CNT;
    #if 0
    if (hencoder->full_turn < 0) {
        degree = (float)(hencoder->full_turn + 1) * 360.0F + ((float)counts * 360.0F) / IMPUSLES_PER_TURN;
    }
    else {
        degree = (float)(hencoder->full_turn - 1) * 360.0F + ((float)counts * 360.0F) / IMPUSLES_PER_TURN;
    }
    #endif
    degree = (float)(hencoder->full_turn) * 360.0F + ((float)counts * 360.0F) / IMPUSLES_PER_TURN;
    return degree;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint32_t time_elapsed_left, time_elapsed_right;
    if (htim->Instance == encoder_left.tim->Instance) {
        overflow_underflow_handler((encoder_t*)&encoder_left);
        /* count RPM*/
        encoder_left.velo = HAL_GetTick() - time_elapsed_left;
        time_elapsed_left = HAL_GetTick();
        /**/
    }
    else if (htim->Instance == encoder_right.tim->Instance)
    {
        overflow_underflow_handler((encoder_t*)&encoder_right);
        /* count RPM */
        encoder_right.velo = HAL_GetTick() - time_elapsed_right;
        time_elapsed_right = HAL_GetTick();
        /* */
    }
}

static inline void overflow_underflow_handler(encoder_t *hencoder) {
    uint16_t count = (uint16_t) __HAL_TIM_GetCounter(hencoder->tim);
    if (count > IMPUSLES_PER_TURN/2){
        hencoder->full_turn--;
    } else {
        hencoder->full_turn++;
    }
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}
