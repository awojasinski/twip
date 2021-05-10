#ifndef CLI_H
#define CLI_H

#define CLI_BUFFER_SIZE 256
#define EOF_BYTE_SET " \r\n"

typedef enum {
    CLI_ENABLED,
    CLI_DISABLED,
} cli_status_t;

typedef enum {
    CMD_OK,
    CMD_WRONG_PARAM,
    CMD_UNSUPPORTED,
    CMD_TOO_LONG,
} cmd_error_t;

void cli_main(void);
void cli_init(void);
void cli_rx_byte_handler(char);
void cli_cmd_analyze(char *);
void cli_printf(const char*, ...);
void cli_printf_inline(const char*, ...);
void cli_delay(uint32_t);
void cli_clear_buffer(void);
void USART2_IRQHandler(void);

#endif