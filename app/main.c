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
#include "invensense9250.h"
#include "cli.h"
#include "control.h"
#include "logger.h"
#include "encoder.h"

static inline void scan_i2c(uint32_t start_addr, uint32_t stop_addr);

#define SAMPLE_RATE_HZ 100 // main filter and control loop speed
#define DT 0.01f           // 1/sample_rate

#define LIMIT_ANGLE (40 * DEG_TO_RAD)

void __balance_controller(void);

control_pid_t control_tilt, control_roll;

void main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_RTC_Init();

    MX_TIM1_Init(); // motor control ch1 left motor ch4 right motor

    // uSD card
    MX_SPI1_Init();
    MX_FATFS_Init();

    MX_I2C1_Init(); // IMU sensor
    cli_init();

    logger_init();
    encoder_init(&htim3, &htim2, 0.4f);

    control_pid_set(&control_tilt, -250.5f, 0.0f, -150.5f, 4, DT, CONTROLER_PD);
    control_enable_saturation(&control_tilt, -100.0f, 100.0f);

    control_pid_set(&control_roll, 0.1f, 0.0f, 0.0f, 4, DT, CONTROLER_P);
    control_enable_saturation(&control_roll, -0.07f, 0.07f);

    control_init(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_4);

    mpu9250_init();
    inv_set_callback(&__balance_controller);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    while (1)
    {
        // cli_main();
        logger_main();
    }
}

void __balance_controller(void)
{
    log_t log_data;
    int8_t control;
    float angle_l, angle_r, wheels_angle, tilt;

    long euler[3];
    float accel[3], gyro[3];

    inv_get_sensor_type_accel(log_data.acc, NULL, NULL);
    inv_get_sensor_type_gyro(log_data.gyro, NULL, NULL);
    inv_get_sensor_type_euler(euler, NULL, NULL);

    angle_l = encoder_get_angle_deg(&encoder_left);  // deg
    angle_r = encoder_get_angle_deg(&encoder_right); // deg

    // Wheel angle
    log_data.angle_l = (int32_t)(angle_l * 128);
    log_data.angle_r = (int32_t)(angle_r * 128);

    tilt = (((float)euler[1] / 65536.f)) * (float)DEG_TO_RAD + 0.03f;
    wheels_angle = ((angle_r + angle_l) / 2.f) * (float)DEG_TO_RAD + tilt;

    // Compute control
    control = control_signal_get(&control_roll, 0 - wheels_angle);
    control = control_signal_get(&control_tilt, 0 - control);

    // Limit control
    if (tilt > LIMIT_ANGLE || tilt < -LIMIT_ANGLE)
    {
        control = 0;
    }

    control_dirve_motors(control, CONTROL_RIGHT_WHEEL);
    control_dirve_motors(control, CONTROL_LEFT_WHEEL);

    log_data.euler = (long)(tilt * 65536.f);
    log_data.control = control;
    log_data.velo_l = (long)(encoder_get_velo(&encoder_left) * 65536.f);
    log_data.velo_r = (long)(encoder_get_velo(&encoder_right) * 65536.f);

    // Add to FIFO
    logger_write(&log_data);
}

void EXTI4_IRQHandler(void)
{
    invensense_interrupt_handler();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

static inline void scan_i2c(uint32_t start_addr, uint32_t stop_addr)
{
    for (uint32_t i = start_addr; i <= stop_addr; i++)
    {
        uint32_t ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 5, 100);
        if (ret == HAL_OK)
        {
            cli_printf("addr: 0x%03x ready", i);
        }
    }
}
