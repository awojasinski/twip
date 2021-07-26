#include <stdarg.h>
#include <stdio.h>
#include <string.h> 

#include "usart.h"
#include "gpio.h"

#include "cli.h"
#include "cli_callbacks.h"
#include "uart.h"

cli_status_t cli_status = CLI_DISABLED;

static volatile char cli_rx_buffer[CLI_BUFFER_SIZE];
static char cli_tx_buffer[CLI_BUFFER_SIZE];

static volatile uint16_t rx_buffer_top = 0;
static volatile bool cmd_analyze = false;

uart_handler_t uart_dbg;

void cli_mute(bool status){
  uart_show_recived_input(!status);
}

void cli_main() {
    if (cmd_analyze) {
        cmd_analyze = false;
        if (strstr((const char*)&cli_rx_buffer, "twip")) {
            cli_cmd_analyze((char *)&cli_rx_buffer);
        }
        cli_clear_buffer();
    }
}

void cli_info() {
  cli_clear_console();
  cli_color_console(TEXT_CYAN_LIGHT);
  cli_printf("++++++++++++++++++++++++++++++++");
  cli_printf("TWIP Firmware");
  cli_printf("Author: Adam Wojasinski");
  cli_printf("Compiled %s %s", __DATE__, __TIME__);
  cli_printf("++++++++++++++++++++++++++++++++");
  cli_color_console(TEXT_DEFAULT);
}

void cli_clear_console() {
  cli_printf_inline("%c[2J", 27);
  cli_printf_inline("%c[H", 27);
}

void cli_color_console(cli_text_color_t color) {
  switch (color) {
    case TEXT_DEFAULT :
      cli_printf_inline("%s", ASCII_COLOR_DEFAULT);
      break;
    case TEXT_RED:
      cli_printf_inline("%s", ASCII_COLOR_RED);
      break;
    case TEXT_RED_LIGHT:
      cli_printf_inline("%s", ASCII_COLOR_RED_LIGHT);
      break;
    case TEXT_BLUE:
      cli_printf_inline("%s", ASCII_COLOR_BLUE);
      break;
    case TEXT_BLUE_LIGHT:
      cli_printf_inline("%s", ASCII_COLOR_BLUE_LIGHT);
      break;
    case TEXT_GREEN:
      cli_printf_inline("%s", ASCII_COLOR_GREEN);
      break;
    case TEXT_GREEN_LIGHT:
      cli_printf_inline("%s", ASCII_COLOR_GREEN_LIGHT);
      break;
    case TEXT_YELLOW:
      cli_printf_inline("%s", ASCII_COLOR_YELLOW);
      break;
    case TEXT_YELLOW_LIGHT:
      cli_printf_inline("%s", ASCII_COLOR_YELLOW_LIGHT);
      break;
    case TEXT_MAGENTA:
      cli_printf_inline("%s", ASCII_COLOR_MAGENTA);
      break;
    case TEXT_MAGENTA_LIGHT:
      cli_printf_inline("%s", ASCII_COLOR_MAGENTA_LIGHT);
      break;
    case TEXT_CYAN:
      cli_printf_inline("%s", ASCII_COLOR_CYAN);
      break;
    case TEXT_CYAN_LIGHT:
      cli_printf_inline("%s", ASCII_COLOR_CYAN_LIGHT);
      break;
    default:
      break;
  }
}

void cli_cmd_analyze(char *buffer) {
  cmd_error_t ret = CMD_UNSUPPORTED;
  uint8_t i = (uint8_t)strcspn(buffer, EOF_BYTE_SET);
  char *cmd = buffer + i + 1;
  while (*cmd != '\0') {
    for (uint8_t n=0; n<CLI_CMD_CALLBACKS_CNT; n++) {
      if (strncmp(cmd, cmd_list[n].command, strlen(cmd_list[n].command)) == 0) {
        ret = cmd_list[n].callback(cmd);
        break;
      }
      ret = CMD_UNSUPPORTED;
    }
    if (ret == CMD_OK) {
      break;
    } else {
      cmd = cmd + 1 + strcspn(cmd, EOF_BYTE_SET);
    }
  }
  switch (ret) {
  case CMD_TOO_LONG:
    cli_printf("%sToo long command%s", ASCII_COLOR_RED_LIGHT, ASCII_COLOR_DEFAULT);
    break;
  case CMD_UNSUPPORTED:
    cli_printf("%sUnsupported command%s", ASCII_COLOR_RED_LIGHT, ASCII_COLOR_DEFAULT);
    break;
  case CMD_WRONG_PARAM:
    cli_printf("%sWrong command or parameter%s", ASCII_COLOR_RED_LIGHT, ASCII_COLOR_DEFAULT);
    break;
  default:
    break;
  }
}

void cli_printf(const char *str, ...) {
  if (cli_status != CLI_ENABLED) {
    return;
  }

  va_list valist;
  va_start(valist, str);
  vsnprintf((char *)&cli_tx_buffer, CLI_BUFFER_SIZE, str, valist);
  uart_send_str(&uart_dbg, cli_tx_buffer);
  uart_send_str(&uart_dbg, "\r\n");
  va_end(valist);
}

void cli_clear_line(uint8_t n) {
  cli_printf_inline("%c[%dA", 27, n);
  cli_printf_inline("%c[0J", 27);
}

void cli_printf_inline(const char *str, ...)
{
  if (cli_status != CLI_ENABLED)
  {
    return;
  }

  va_list valist;
  va_start(valist, str);
  vsnprintf((char *)&cli_tx_buffer, CLI_BUFFER_SIZE, str, valist);
  uart_send_str(&uart_dbg, cli_tx_buffer);
  va_end(valist);
}

void cli_rx_byte_handler(char data) {
  cli_rx_buffer[rx_buffer_top] = data;

  if (data == '\r') {
    uart_send_cnt(&uart_dbg, "\n", 1);
    cli_rx_buffer[rx_buffer_top+1] = '\0';
    cmd_analyze = true;
  } else if (data == '\177') {
    if (rx_buffer_top > 0) {
      rx_buffer_top--;
    }
    cli_rx_buffer[rx_buffer_top] = '\0';
  } else if (rx_buffer_top >= (CLI_BUFFER_SIZE-2)) {
    rx_buffer_top = 0;
  } else {
    rx_buffer_top++;
  }
}

void cli_clear_buffer(void) {
  for (int16_t i=0; i<CLI_BUFFER_SIZE; i++) {
    cli_rx_buffer[i] = 0;
  }
  rx_buffer_top = 0;
}

void cli_delay(uint32_t delay_ms) {
  uint32_t start_tick = HAL_GetTick();
  while (HAL_GetTick() - start_tick < delay_ms) {
    cli_main();
  }
}

char cli_get_char(void) {
  return rx_buffer_top <= 0 ? '\0' : cli_rx_buffer[rx_buffer_top-1];
}

void USART2_IRQHandler(void) {
  if (uart_dbg.huart->Instance == USART2) {
    uart_IRQ(&uart_dbg);
  }
}

void cli_init()
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);;
  }

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
  GPIO_InitStruct.Pin = ST_LINK_RX_Pin | ST_LINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  uart_init(&uart_dbg, &huart2, UART_DBG);
  uart_show_recived_input(true);
  cli_status = CLI_ENABLED;
}