#include <stdbool.h>

#include "usart.h"

#include "uart.h"

static uint16_t uart_check_tx_size(uart_handler_t*);
static bool uart_print_recived_input = true;

void uart_init(uart_handler_t *u, UART_HandleTypeDef *huart, uart_name name) {
  u->huart = huart;
  u->name = name;

  uart_reinit(u);
}

void uart_deinit(uart_handler_t *u) {
  if (u->uart_initialized == true) {
    u->uart_initialized = false;
    __HAL_UART_DISABLE_IT(u->huart, UART_IT_RXNE);
    HAL_UART_DeInit(u->huart);
  }
}

void uart_reinit(uart_handler_t *u) {
  if (u->huart != NULL) {
    fifo_init(&u->uart_tx_fifo, (uint8_t *)&u->tx_buffer, (uint16_t)CLI_BUFFER_SIZE);
    HAL_StatusTypeDef ret = HAL_UART_Init(u->huart);
    if (ret != HAL_OK)
    {
      Error_Handler(__FILE__, __LINE__);
    }
    u->uart_initialized = true;
    __HAL_UART_ENABLE_IT(u->huart, UART_IT_RXNE);
  }
}

void uart_send(uart_handler_t *u, const char *str, uint8_t cnt) {
  if (u->uart_initialized != true) {
    return;
  }
  
  uint8_t data_cnt = 0;
  uint32_t timeout;

  if (cnt == 0) {
    while (*str != '\0') {
      data_cnt++;
      timeout = 0;
      while (uart_check_tx_size(u) >= u->uart_tx_fifo.size - 1) {
        if (++timeout > 100000) {
          break;
        }
      }
      fifo_write(&u->uart_tx_fifo, str, 1);
      str++;
    }
  } else {
    for (uint8_t i=0; i<cnt; i++) {
      data_cnt++;
      timeout = 0;
      while (uart_check_tx_size(u) >= u->uart_tx_fifo.size - 1)
      {
        if (++timeout > 100000)
        {
          break;
        }
      }
      fifo_write(&u->uart_tx_fifo, str, 1);
      str++;
    }
  }

  if (data_cnt > 0) {
    __HAL_UART_ENABLE_IT(u->huart, UART_IT_TXE);
  }
}

static uint16_t uart_check_tx_size(uart_handler_t *u) {
  return fifo_data_size(&(u->uart_tx_fifo));
}

void uart_IRQ(uart_handler_t *u) {
  // UART RX
  if (u->huart->Instance->ISR & USART_ISR_RXNE_RXFNE)
  {
    char rx_data = 0;
    rx_data = (char)(u->huart->Instance->RDR);
    cli_rx_byte_handler(rx_data);
    if (uart_print_recived_input) {
      uart_send_cnt(u, (char *)&rx_data, 1);
    }
  }

  // UART TX FIFO
  if (__HAL_UART_GET_IT_SOURCE(u->huart, UART_IT_TXE) != RESET)
  {
    /* Write 1 byte to the transmit data register */
    char data_read = 0;
    uint8_t nr_bytes = 0;
    nr_bytes = (uint8_t)fifo_read(&u->uart_tx_fifo, &data_read, 1);
    if (nr_bytes > 0)
    {
      u->huart->Instance->TDR = data_read;
    }
    else
    {
      __HAL_UART_DISABLE_IT(u->huart, UART_IT_TXE);
    }
  }

  static uint32_t SR_reg = 0, DR_reg;
  SR_reg = u->huart->Instance->ISR; //clear ore sequence
  DR_reg = u->huart->Instance->RDR; //clear ore sequence
  UNUSED(DR_reg);
  if (SR_reg & UART_FLAG_ORE)
  {
    __HAL_UART_SEND_REQ(u->huart, USART_RQR_RXFRQ);
    __HAL_UART_CLEAR_FLAG(u->huart, USART_ICR_ORECF);
  }
}

void uart_show_recived_input(bool enable) {
  uart_print_recived_input = enable;
}