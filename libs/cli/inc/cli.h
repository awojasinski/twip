#ifndef CLI_H
#define CLI_H

#define CLI_BUFFER_SIZE 256
#define EOF_BYTE_SET " \r\n"

#define ASCII_COLOR_DEFAULT         "\033[0;39m"
#define ASCII_COLOR_RED             "\033[0;31m"
#define ASCII_COLOR_RED_LIGHT       "\033[1;31m"
#define ASCII_COLOR_BLUE            "\033[0;34m"
#define ASCII_COLOR_BLUE_LIGHT      "\033[1;34m"
#define ASCII_COLOR_GREEN           "\033[0;32m"
#define ASCII_COLOR_GREEN_LIGHT     "\033[1;32m"
#define ASCII_COLOR_YELLOW          "\033[0;33m"
#define ASCII_COLOR_YELLOW_LIGHT    "\033[1;33m"
#define ASCII_COLOR_CYAN            "\033[0;36m"
#define ASCII_COLOR_CYAN_LIGHT      "\033[1;36m"
#define ASCII_COLOR_MAGENTA         "\033[0;35m"
#define ASCII_COLOR_MAGENTA_LIGHT   "\033[1;35m"

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

typedef enum {
    TEXT_DEFAULT,
    TEXT_RED,
    TEXT_RED_LIGHT,
    TEXT_GREEN,
    TEXT_GREEN_LIGHT,
    TEXT_BLUE,
    TEXT_BLUE_LIGHT,
    TEXT_CYAN,
    TEXT_CYAN_LIGHT,
    TEXT_MAGENTA,
    TEXT_MAGENTA_LIGHT,
    TEXT_YELLOW,
    TEXT_YELLOW_LIGHT,
} cli_text_color_t;

void cli_main(void);
void cli_init(void);
void cli_info(void);
void cli_clear_console(void);
void cli_color_console(cli_text_color_t);
void cli_clear_line(uint8_t);
void cli_rx_byte_handler(char);
void cli_cmd_analyze(char *);
void cli_printf(const char*, ...);
void cli_printf_inline(const char*, ...);
void cli_delay(uint32_t);
void cli_clear_buffer(void);
void USART2_IRQHandler(void);

#endif