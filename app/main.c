#include "string.h"

#include "main.h"
#include "app_fatfs.h"
#include "cordic.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//#include "ina219.h"
//#include "mpu9250.h"
#include "cli.h"
#include "control.h"
#include "logger.h"
#include "encoder.h"
#include "invensense9250.h"

log_t m_log_data[2][LOG_BUFFER];

void main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_RTC_Init();

    MX_TIM1_Init(); // motor control ch1 left motor ch4 right motor
    MX_TIM4_Init(); // control algorithm

    // uSD card
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_FATFS_Init();

    fatfs_test();

    MX_I2C1_Init(); // IMU sensor

    cli_init();

    encoder_init(&htim3, &htim2, 40);
    mpu9250_init();

    control_state_set(0, 0, 0, 0, 0, 0);
    control_pid_set(&pid_pitch, 10, 0, 0);
    control_pid_set(&pid_roll, 0, 0, 0);

    control_init(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_4, 5.7f);
    logger_init();

    HAL_TIM_Base_Start_IT(&htim4);
    while (1)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(10);
        cli_main();
    }
}

void TIM4_IRQHandler(void)
{
    // Get measurements
    static int buffer_idx = 0;
    static int sample = 0;

    int8_t control_l, control_r;
    control_state_t twip_state;

    inv_get_accel(m_log_data[buffer_idx][sample].acc);
    inv_get_gyro(m_log_data[buffer_idx][sample].gyro);
    inv_get_gyro_set_raw(m_log_data[buffer_idx][sample].gyro_raw, NULL, NULL);
    inv_get_sensor_type_euler(m_log_data[buffer_idx][sample].euler, NULL, NULL);

    m_log_data[buffer_idx][sample].angle_l = (int32_t)encoder_get_angle_deg(&encoder_left) * 65536;
    m_log_data[buffer_idx][sample].angle_r = (int32_t)encoder_get_angle_deg(&encoder_right) * 65536;

    // Calculate control
    twip_state.pitch = inv_q16_to_float(m_log_data[buffer_idx][sample].euler[1]);
    twip_state.yaw = inv_q16_to_float(m_log_data[buffer_idx][sample].euler[2]);
    twip_state.roll = (encoder_get_angle_deg(&encoder_right) + encoder_get_angle_deg(&encoder_left)) / 2;

    control_signal_get(&control_r, &control_l, &twip_state);
    control_dirve_motors(control_r, CONTROL_RIGHT_WHEEL);
    control_dirve_motors(control_l, CONTROL_LEFT_WHEEL);

    m_log_data[buffer_idx][sample].control_r = control_r;
    m_log_data[buffer_idx][sample].control_l = control_l;

    sample++;
    if (sample == LOG_BUFFER)
    {
        save_log(m_log_data[buffer_idx]);
        sample = 0;
        buffer_idx = ~buffer_idx & 0x1;
    }

    HAL_TIM_IRQHandler(&htim4);
}
