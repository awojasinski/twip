#include "tim.h"
#include "encoder.h"
#include "fir.h"

#define MOVING_AVERAGE_SIZE 5
#define IMP2RAD ((2 * 3.1415f) / (IMPUSLES_PER_TURN)*200) //rad/s

static inline void overflow_underflow_handler(encoder_t *);

encoder_t encoder_right, encoder_left;

int32_t r_velo_ma[MOVING_AVERAGE_SIZE];
int32_t l_velo_ma[MOVING_AVERAGE_SIZE];

fir_ma_filter_t r_filter, l_filter;

void encoder_init(TIM_HandleTypeDef *left_tim, TIM_HandleTypeDef *right_tim, float wheel_diameter_m)
{
    MX_TIM2_Init(); // encoder right motor
    MX_TIM3_Init(); // encoder left motor
    MX_TIM7_Init(); // for valocity calculation

    encoder_left.tim = left_tim;
    encoder_right.tim = right_tim;

    encoder_left.velo = 0;

    encoder_left.full_turn = -1;
    encoder_right.full_turn = 0;

    encoder_left.wheel_r = wheel_diameter_m;
    encoder_right.wheel_r = wheel_diameter_m;

    fir_ma_init(&r_filter, r_velo_ma, MOVING_AVERAGE_SIZE);
    fir_ma_init(&l_filter, l_velo_ma, MOVING_AVERAGE_SIZE);

    HAL_TIM_Encoder_Start(encoder_left.tim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(encoder_right.tim, TIM_CHANNEL_ALL);

    HAL_TIM_Base_Start_IT(encoder_left.tim);
    HAL_TIM_Base_Start_IT(encoder_right.tim);
    HAL_TIM_Base_Start_IT(&htim7);
}

float encoder_get_angle_deg(encoder_t *hencoder)
{
    float degree;
    uint32_t counts = hencoder->tim->Instance->CNT;
    degree = (float)(hencoder->full_turn) * 360.0F + ((float)counts * 360.0F) / IMPUSLES_PER_TURN;
    return degree;
}

float encoder_get_velo(encoder_t *hencoder)
{
    if (hencoder == &encoder_left)
    {
        return ((float)fir_ma_read(&l_filter) * IMP2RAD);
    }
    else if (hencoder == &encoder_right)
    {
        return ((float)fir_ma_read(&r_filter) * IMP2RAD);
    }
    else
    {
        return -1.f;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t time_elapsed_left, time_elapsed_right;
    if (htim == encoder_left.tim)
    {
        uint32_t timestamp = HAL_GetTick();
        overflow_underflow_handler((encoder_t *)&encoder_left);
        encoder_left.velo = (1000 * 2 * 3.1415f) / (float)(timestamp - time_elapsed_left); // rad/s
        time_elapsed_left = timestamp;
    }
    else if (htim == encoder_right.tim)
    {
        uint32_t timestamp = HAL_GetTick();
        overflow_underflow_handler((encoder_t *)&encoder_right);
        encoder_right.velo = (1000 * 2 * 3.1415f) / (float)(timestamp - time_elapsed_right); // rad/s
        time_elapsed_right = timestamp;
    }
}

static inline void overflow_underflow_handler(encoder_t *hencoder)
{
    uint16_t count = (uint16_t)__HAL_TIM_GetCounter(hencoder->tim);
    if (count > IMPUSLES_PER_TURN / 2)
    {
        hencoder->full_turn--;
    }
    else
    {
        hencoder->full_turn++;
    }
}

void TIM7_IRQHandler(void)
{
    encoder_right.encoder_cnt_new = (int32_t)__HAL_TIM_GET_COUNTER(encoder_right.tim) +
                                    (int32_t)(encoder_right.full_turn * (int32_t)__HAL_TIM_GET_AUTORELOAD(encoder_right.tim));
    encoder_left.encoder_cnt_new = (int32_t)__HAL_TIM_GET_COUNTER(encoder_left.tim) +
                                   (int32_t)(encoder_left.full_turn * (int32_t)__HAL_TIM_GET_AUTORELOAD(encoder_left.tim));

    int32_t r_diff = encoder_right.encoder_cnt_new - encoder_right.encoder_cnt_old;
    int32_t l_diff = encoder_left.encoder_cnt_new - encoder_left.encoder_cnt_old;

    fir_ma_update(&r_filter, r_diff);
    fir_ma_update(&l_filter, l_diff);

    encoder_right.encoder_cnt_old = encoder_right.encoder_cnt_new;
    encoder_left.encoder_cnt_old = encoder_left.encoder_cnt_new;

    HAL_TIM_IRQHandler(&htim7);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}
