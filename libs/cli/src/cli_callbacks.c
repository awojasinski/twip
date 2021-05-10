#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "gpio.h"

#include "cli.h"
#include "cli_callbacks.h"

static cmd_error_t cli_pwm_callback(char*);
static cmd_error_t cli_led_callback(char*);
static cmd_error_t cli_pause_callback(char*);
static cmd_error_t cli_continue_callback(char*);

static bool twip_paused = false;

const cmd_t cmd_list[CALLBACKS_CNT] = {
  {CLI_CALLBACK_PWM, "pwm", &cli_pwm_callback},
  {CLI_CALLBACK_LED, "led", &cli_led_callback},
  {CLI_CALLBACK_PAUSE, "pause", &cli_pause_callback},
  {CLI_CALLBACK_CONTINUE, "continue", &cli_continue_callback},
};

static cmd_error_t cli_pwm_callback(char *cmd) {
    strtok(cmd, EOF_BYTE_SET);
    char *param = strtok(NULL, EOF_BYTE_SET);

    uint8_t pwm_filling = (uint8_t)atoi(param);
    if (pwm_filling <= 100) {
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
    }
    return CMD_WRONG_PARAM;
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