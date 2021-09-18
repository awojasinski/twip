#include "main.h"
#include "app_fatfs.h"
#include "cordic.h"
#include "fmac.h"
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

void main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_RTC_Init();

    MX_TIM1_Init(); // motor control ch1 left motor ch4 right motor
    MX_TIM4_Init(); // control algorithm

    // uSD card
    MX_SPI1_Init();
    MX_FATFS_Init();

    MX_I2C1_Init(); // IMU sensor

    cli_init();

    encoder_init(&htim3, &htim2, 40);
    mpu9250_init();

    control_state_set(0, 0, 0, 0, 0, 0);
    control_pid_set(&pid_pitch, 5, 0, 0);
    control_pid_set(&pid_roll, 0, 0, 0);
    control_init(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_4);

    logger_init();

    HAL_TIM_Base_Start_IT(&htim4);
    while (1)
    {
        cli_main();
        logger_main();
        //cli_printf("%7.2f", encoder_get_angle_deg(&encoder_left));
    }
}

void TIM4_IRQHandler(void)
{
    // Get measurements
    log_t log_data;
    int8_t control;
    control_state_t twip_state;
    float angle_l, angle_r;

    inv_get_accel(log_data.acc);
    inv_get_gyro(log_data.gyro);
    inv_get_sensor_type_euler(log_data.euler, NULL, NULL);

    angle_l = encoder_get_angle_deg(&encoder_left);
    angle_r = encoder_get_angle_deg(&encoder_right);

    log_data.angle_l = (int32_t)(angle_l * 128);
    log_data.angle_r = (int32_t)(angle_r * 128);

    // Calculate control
    twip_state.pitch = inv_q16_to_float(log_data.euler[1]);
    twip_state.roll = (angle_r + angle_l) / 2;
    twip_state.droll = (encoder_get_velo(&encoder_left) + encoder_get_velo(&encoder_right)) / 2;

    //control_signal_get(&control, &control, &twip_state);
    //control_dirve_motors(control, CONTROL_RIGHT_WHEEL);
    //control_dirve_motors(control, CONTROL_LEFT_WHEEL);

    log_data.control = (int8_t)htim1.Instance->CCR1;
    log_data.euler[0] = (long)(encoder_get_velo(&encoder_left) * 65536.f);
    log_data.euler[1] = (long)(encoder_get_velo(&encoder_right) * 65536.f);
    log_data.acc[0] = (int8_t)((htim1.Instance->CCR1 / htim1.Init.Period) * 100);
    log_data.acc[1] = (int8_t)((htim1.Instance->CCR4 / htim1.Init.Period) * 100);
    log_data.velocity = (long)(twip_state.droll * 65536.f);

    // Add to FIFO
    logger_write(&log_data);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    HAL_TIM_IRQHandler(&htim4);
}
