#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <string.h>

#include "app_fatfs.h"

#define LOG_BUFFER 100

typedef struct
{
    long acc[3];
    long gyro[3];
    long gyro_raw[3];
    long euler[3];
    int8_t control_l;
    int8_t control_r;
    long angle_l;
    long angle_r;
} log_t;

extern FATFS FileSystem;
extern FIL logger_file;

extern volatile bool sd_card_inserted;

void logger_init(void);
void logger_deinit(void);
void save_log(log_t const *);
FRESULT log_data(char *, UINT);
void logger_sd_card_detection_irq(void);

void fatfs_test(void);
void EXTI9_5_IRQHandler(void);

#endif
