#ifndef UART_H
#define UART_H

#include "stdbool.h"

#include "usart.h"

#include "cli.h"
#include "fifo.h"

#define uart_send_str(uart, str)      uart_send(uart, str, 0)
#define uart_send_cnt(uart, str, cnt) uart_send(uart, str, cnt)

typedef enum {
    UART_DBG,
    UART_ESP,
} uart_name;

typedef struct {
    UART_HandleTypeDef *huart;
    volatile uint8_t tx_buffer[256];
    fifo_t uart_tx_fifo;
    bool uart_initialized;
    uart_name name;
} uart_handler_t;

void uart_init(uart_handler_t*, UART_HandleTypeDef*, uart_name);
void uart_IRQ(uart_handler_t*);
void uart_deinit(uart_handler_t*);
void uart_reinit(uart_handler_t*);
void uart_send_str_blocking(uart_handler_t*, const char*);
void uart_send(uart_handler_t *, const char *, uint8_t);

#endif