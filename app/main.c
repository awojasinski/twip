#include "main.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "cli.h"
#include "mpu9250.h"

void main(void) {

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init(); //ESP32
  MX_SPI1_Init();
  MX_FATFS_Init();

  cli_init();
  cli_info();
  mpu9250_init();

  while (1) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    cli_delay(200);
  }
}