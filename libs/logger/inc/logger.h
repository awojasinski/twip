#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>

#include "app_fatfs.h"

extern FATFS FileSystem;
extern FIL logger_file;

extern volatile bool sd_card_inserted;

void logger_init(void);
void logger_deinit(void);
FRESULT log_data(char*, UINT);
void logger_sd_card_detection_irq(void);

void EXTI9_5_IRQHandler(void);

#endif