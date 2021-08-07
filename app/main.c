#include "string.h"

#include "main.h"
#include "app_fatfs.h"
#include "cordic.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//#include "ina219.h"
//#include "mpu9250.h"
#include "cli.h"
#include "logger.h"
#include "encoder.h"
#include "invensense9250.h"

static inline void scan_i2c(I2C_HandleTypeDef *, uint16_t, uint16_t);

void main(void) {

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  //MX_USART1_UART_Init(); //ESP32
  //MX_SPI1_Init();
  //MX_FATFS_Init();
  MX_I2C1_Init();
  
  cli_init();
  cli_mute(false);
  cli_info();

  HAL_Delay(100);
  //scan_i2c(&hi2c1, 0, 0x0070);

  encoder_init(&htim3, &htim2, 90);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);

  //fatfs_test();
  mpu9250_init();
  //run_self_test();

  //HAL_SPI_MspDeInit(&hspi1);
  //logger_init();
  long accel[3], gyro[3], euler[3], lquat[4], heading;
  float fquat[4];
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    //pulses = htim3.Instance->CNT;
    inv_get_accel(accel);
    inv_get_gyro(gyro);
    cli_printf("gyro: %7.4f %7.4f %7.4f",
               inv_q16_to_float(gyro[0]),
               inv_q16_to_float(gyro[1]),
               inv_q16_to_float(gyro[2]));
    cli_printf("accel: %7.4f %7.4f %7.4f",
               inv_q16_to_float(accel[0]),
               inv_q16_to_float(accel[1]),
               inv_q16_to_float(accel[2]));
    inv_get_6axis_quaternion(lquat);
      cli_printf("lquat: %7.4f %7.4f %7.4f %7.4f",
                 inv_q30_to_float(lquat[0]),
                 inv_q30_to_float(lquat[1]),
                 inv_q30_to_float(lquat[2]),
                 inv_q30_to_float(lquat[3]));
    inv_get_sensor_type_euler(euler, NULL, NULL);
      cli_printf("euler: %7.4f %7.4f %7.4f",
                 inv_q16_to_float(euler[0]),
                 inv_q16_to_float(euler[1]),
                 inv_q16_to_float(euler[2]));
    inv_get_sensor_type_heading(&heading, NULL, NULL);
    cli_printf("heading: %7.4f", inv_q16_to_float(heading));
    inv_get_quaternion_float(fquat);
      cli_printf("fquat: %7.4f %7.4f %7.4f %7.4f",
                 fquat[0],
                 fquat[1],
                 fquat[2],
                 fquat[3]);
    //cli_printf("Pulses: %4u Angle: %6.2f Velo: %6.2f", pulses, encoder_get_angle_deg((encoder_t*)&encoder_left), encoder_left.ang_velo);
    cli_delay(100);
    cli_clear_line(6);
  }
}

static inline void scan_i2c(I2C_HandleTypeDef *i2c, uint16_t start_addr, uint16_t stop_addr) {
  cli_printf("Scan I2C addresses");
  for (uint16_t addr = start_addr; addr <= stop_addr; addr++) {
    if( HAL_I2C_IsDeviceReady(i2c, addr << 1, 5, 200) == HAL_OK) {
      cli_printf_inline("%s0x%04X%s, ", ASCII_COLOR_GREEN_LIGHT, addr, ASCII_COLOR_DEFAULT);
    } else {
      cli_printf_inline("%s0x%04X%s, ", ASCII_COLOR_RED_LIGHT, addr, ASCII_COLOR_DEFAULT);
    }
    if ((addr-start_addr+1)%10 == 0) {
      cli_printf_inline("\n\r");
    }
  }
  cli_printf_inline("\n\r");
  HAL_Delay(10);
}
