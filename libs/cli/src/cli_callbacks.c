#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "gpio.h"
#include "tim.h"

#include "cli.h"
#include "cli_callbacks.h"
#include "uart.h"
#include "mpu9250.h"

static cmd_error_t cli_pwm_callback(char*);
static cmd_error_t cli_led_callback(char*);
static cmd_error_t cli_pause_callback(char*);
static cmd_error_t cli_continue_callback(char*);
static cmd_error_t cli_help_callback(char*);
static cmd_error_t cli_cls_callback(char*);
static cmd_error_t cli_imu_callback(char*);

static bool twip_paused = false;

const cmd_t cmd_list[CALLBACKS_CNT] = {
  {CLI_CALLBACK_PWM, "pwm", &cli_pwm_callback},
  {CLI_CALLBACK_LED, "led", &cli_led_callback},
  {CLI_CALLBACK_PAUSE, "pause", &cli_pause_callback},
  {CLI_CALLBACK_CONTINUE, "continue", &cli_continue_callback},
  {CLI_CALLBACK_HELP, "help", &cli_help_callback},
  {CLI_CALLBACK_CLEAR, "cls", &cli_cls_callback},
  {CLI_CALLBACK_IMU, "imu", &cli_imu_callback},
};

static cmd_error_t cli_pwm_callback(char *cmd) {
  strtok(cmd, EOF_BYTE_SET);
  char *param = strtok(NULL, EOF_BYTE_SET);

  uint8_t pwm_filling = (uint8_t)atoi(param);
  if (pwm_filling <= 100) {
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)pwm_filling*htim1.Init.Period/100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    return CMD_OK;
  } else {
    return CMD_WRONG_PARAM;
  }
}

static cmd_error_t cli_led_callback(char *cmd) {
    strtok(cmd, EOF_BYTE_SET);
    char *cmd_chunk = strtok(NULL, EOF_BYTE_SET);

    if (strcmp(cmd_chunk, "blink") == 0) {
        char *param = strtok(NULL, EOF_BYTE_SET);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay((uint32_t)atoi(param));
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        return CMD_OK;
    } else if (strcmp(cmd_chunk, "power") == 0) {
        char *param = strtok(NULL, EOF_BYTE_SET);
        if (strcmp(param, "on") == 0 || strcmp(param, "1") == 0) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            return CMD_OK;
        } else if (strcmp(param, "off") == 0 || strcmp(param, "0") == 0) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            return CMD_OK;
        }
        return CMD_WRONG_PARAM;
    } else if (stpcpy(cmd_chunk, "on") == 0) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      return CMD_OK;
    } else if (stpcpy(cmd_chunk, "off") == 0) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
      return CMD_WRONG_PARAM;
}

static cmd_error_t cli_help_callback(char *cmd) {
  UNUSED(cmd);
  cli_info();
  cli_printf("Available commands");
  for (uint8_t i=0; i<CLI_CMD_CALLBACKS_CNT; i++) {
    cli_printf("twip %s", cmd_list[i].command);
  }
  return CMD_OK;
}

static cmd_error_t cli_cls_callback(char *cmd) {
  UNUSED(cmd);
  cli_clear_console();
  cli_info();
  return CMD_OK;
}

static cmd_error_t cli_imu_callback(char *cmd) {
  UNUSED(cmd);
  uart_show_recived_input(false);
  cli_clear_line(1);
  char c;
  do {
    mpu9250_data_scaled((mpu9250_data_t *)&hmpu9250_data);
    cli_printf("ACCEL:\tX:%6.3f Y:%6.3f Z:%6.3f", hmpu9250_data.accel.x, hmpu9250_data.accel.y, hmpu9250_data.accel.z);
    cli_printf("GYRO:\tX:%6.3f Y:%6.3f Z:%6.3f", hmpu9250_data.gyro.x, hmpu9250_data.gyro.y, hmpu9250_data.gyro.z);
    cli_printf("TEMP: %4.2f", hmpu9250_data.temp);

    cli_delay(200);
    cli_clear_line(3);
    c = cli_get_char();
  } while (c != 'q' && c != 'Q' && c != 27);
  cli_clear_buffer();
  uart_show_recived_input(true);

  return CMD_OK;
}

static cmd_error_t cli_pause_callback(char *cmd) {
  UNUSED(cmd);
  twip_paused = true;
  cli_printf("PAUSED");
  cli_clear_buffer();
  while (twip_paused) {
    cli_main();
  }
  return CMD_OK;
}

static cmd_error_t cli_continue_callback(char *cmd) {
  UNUSED(cmd);
  twip_paused = false;
  cli_printf("CONTINUE");
  return CMD_OK;
}