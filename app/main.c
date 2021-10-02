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

#define DEG2RAD (3.14f / 180.f)
#define RAD2DEG (180.f / 3.14f)
#define G2MS (9.81)

#define LIMIT_ANGLE (40 * DEG2RAD)

void main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_RTC_Init();

    MX_TIM1_Init(); // motor control ch1 left motor ch4 right motor
    //MX_TIM4_Init(); // control algorithm

    // uSD card
    MX_SPI1_Init();
    MX_FATFS_Init();

    MX_I2C1_Init(); // IMU sensor

    cli_init();
    cli_mute(true);

    //encoder_init(&htim3, &htim2, 0.4f);
    //mpu9250_init();

    //control_state_set(0, 0, 0, 0, 0, 0);
    //control_pid_set(&pid_pitch, 73.5f, 0, 0.0f);
    //control_pid_set(&pid_roll, 0.f, 0, 0.9f);
    //control_init(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_4);

    //inv_get_sensor_type_gyroscope
    logger_init();

    //HAL_TIM_Base_Start_IT(&htim4);
    cli_printf("This will sample the magnetometer for the next 15 seconds\n");
    cli_printf("Rotate the board around in the air through as many orientations\n");
    cli_printf("as possible to collect sufficient data for calibration\n");
    cli_printf("Press any key to continue\n");
    cli_getchar();

    cli_printf("spin spin spin!!!\n\n");
    // wait for the user to actually start
    HAL_Delay(2000);

    if (calibrate_mag() < 0)
    {
        cli_printf("Failed to complete magnetometer calibration\n");
    }
    else
    {
        cli_printf("Calibrated");
    }

    while (1)
    {
        //cli_main();
        //logger_main();
    }
}

void TIM4_IRQHandler(void)
{
    // Get measurements
    log_t log_data;
    int8_t control;
    control_state_t twip_state;
    long euler[3];
    float angle_l, angle_r;

    inv_get_accel(log_data.acc);
    inv_get_gyro(log_data.gyro);
    inv_get_sensor_type_euler(euler, NULL, NULL);

    log_data.euler = euler[1];

    angle_l = encoder_get_angle_deg(&encoder_left);  // deg
    angle_r = encoder_get_angle_deg(&encoder_right); // deg

    // Wheel angle
    log_data.angle_l = (int32_t)(angle_l * 128); // 1 deg = 2^8
    log_data.angle_r = (int32_t)(angle_r * 128); // 1 deg = 2^8

    // Calculate control
    twip_state.pitch = (inv_q16_to_float(euler[1])) * DEG2RAD;
    twip_state.dpitch = (inv_q16_to_float(log_data.gyro[1])) * DEG2RAD;
    twip_state.roll = ((angle_r + angle_l) / 2) * DEG2RAD;
    twip_state.droll = (encoder_get_velo(&encoder_left) + encoder_get_velo(&encoder_right)) / 2;

    // Limit control
    control_signal_get(&control, &control, &twip_state);
    if (twip_state.pitch > LIMIT_ANGLE || twip_state.pitch < -LIMIT_ANGLE)
    {
        control = 0;
        control_errors_clear();
    }

    control_dirve_motors(control, CONTROL_RIGHT_WHEEL);
    control_dirve_motors(control, CONTROL_LEFT_WHEEL);

    log_data.control = control;
    log_data.velo_l = (long)(encoder_get_velo(&encoder_left) * 65536.f);
    log_data.velo_r = (long)(encoder_get_velo(&encoder_right) * 65536.f);

    // Add to FIFO
    logger_write(&log_data);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_TIM_IRQHandler(&htim4);
}
