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

#include "cli.h"
#include "mpu9250.h"
#include "logger.h"
#include "encoder.h"
#include "ina219.h"

volatile uint32_t pulses;

static inline void fatfs_test(void);
//static void scan_i2c(I2C_HandleTypeDef*, uint16_t, uint16_t);

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
  cli_info();

  HAL_Delay(100);

  //scan_i2c(&hi2c1, 0, 0x100);

  encoder_init(&htim3, &htim2, 90);
  //ina219_init(&hi2c1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);

  //fatfs_test();
  //mpu9250_init();
  //fatfs_test();
  //HAL_SPI_MspDeInit(&hspi1);
  //logger_init();
  cli_mute(false);

  while (1) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    pulses = htim3.Instance->CNT;
    //cli_printf("Pulses: %4u Angle: %6.2f Velo: %6.2f", pulses, encoder_get_angle_deg((encoder_t*)&encoder_left), encoder_left.ang_velo);
    cli_printf("PWM:%d RPM:%lu", __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1)/10, encoder_left.velo); //encoder_get_angle_deg(&encoder_left));
    //cli_printf("%")
    cli_delay(10);
    cli_clear_line(1);
  }
}

/*
static void scan_i2c(I2C_HandleTypeDef *i2c, uint16_t start_addr, uint16_t stop_addr) {
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
*/

static inline void fatfs_test(){
  FATFS FS;
  FIL f;
  FRESULT res;

  res = f_mount(&FS, "", 1); //1=mount now
  if (res != FR_OK)
  {
    cli_printf("f_mount error (%i)\r\n", res);
    while (1)
      ;
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS *getFreeFs;

  res = f_getfree("", &free_clusters, &getFreeFs);
  if (res != FR_OK)
  {
    cli_printf("f_getfree error (%i)\r\n", res);
    while (1)
      ;
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  cli_printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  //Now let's try to open fe "test.txt"
  res = f_open(&f, "test.txt", FA_READ);
  if (res != FR_OK)
  {
    cli_printf("f_open error (%i)\r\n");
    while (1)
      ;
  }
  cli_printf("I was able to open 'test.txt' for reading!\r\n");

  //Read 30 bytes from "test.txt" on the SD card
  BYTE readBuf[30];

  //We can either use f_read OR f_gets to get data out of fes
  //f_gets is a wrapper on f_read that does some string formatting for us
  TCHAR *rres = f_gets((TCHAR *)readBuf, 30, &f);
  if (rres != 0)
  {
    cli_printf("Read string from 'test.txt' contents: %s\r\n", readBuf);
  }
  else
  {
    cli_printf("f_gets error (%i)\r\n", res);
  }

  //Be a tidy kiwi - don't forget to close your fe!
  f_close(&f);

  //Now let's try and write a fe "write.txt"
  res = f_open(&f, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if (res == FR_OK)
  {
    cli_printf("I was able to open 'write.txt' for writing\r\n");
  }
  else
  {
    cli_printf("f_open error (%i)\r\n", res);
  }

  //Copy in a string
  strncpy((char *)readBuf, "a new file is made!", 19);
  UINT bytesWrote;
  res = f_write(&f, readBuf, 19, &bytesWrote);
  if (res == FR_OK)
  {
    cli_printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
  }
  else
  {
    cli_printf("f_write error (%i)\r\n");
  }

  //Be a tidy kiwi - don't forget to close your fe!
  f_close(&f);

  //We're done, so de-mount the drive
  f_mount(NULL, "", 1);
}