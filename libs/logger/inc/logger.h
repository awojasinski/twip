#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include "app_fatfs.h"

#define LOG_FIFO_SIZE 160

typedef struct
{
    long acc[3];
    long gyro[3];
    long euler;
    long angle_l;
    long angle_r;
    long velo_l;
    long velo_r;
    int8_t control;
} log_t;

typedef struct
{
    size_t head;
    size_t tail;
    size_t size;
    log_t **data;
} log_fifo_t;

extern FATFS FileSystem;
extern FIL logger_file;

extern volatile bool logger_initialized;

void logger_init(void);
void logger_deinit(void);

void logger_main(void);
int8_t logger_write(log_t *);
log_t *logger_read(void);

void fatfs_test(void);
void EXTI9_5_IRQHandler(void);

#endif
