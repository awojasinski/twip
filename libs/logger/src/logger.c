#include "stm32g4xx_hal.h"

#include "cli.h"
#include "logger.h"

FATFS FileSystem;
FIL logger_file;
volatile FRESULT logger_result;

volatile bool sd_card_inserted = false;
volatile bool log_file_created = false;

void logger_init() {
    if (!sd_card_inserted) {
        if (HAL_GPIO_ReadPin(SD_card_detection_GPIO_Port, SD_card_detection_Pin) == GPIO_PIN_SET)
        {
            if (f_mount((FATFS *)&FileSystem, "", 1) == FR_OK)
            {
                sd_card_inserted = true;
                cli_printf("SD card detected!");
                if (f_open(&logger_file, "log.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS) == FR_OK) {
                    log_file_created = true;
                }
            }
        }
    }
}

void logger_deinit() {
    if (sd_card_inserted) {
        sd_card_inserted = false;
        if (f_close(&logger_file) == FR_OK) {
            log_file_created = false;
        };
        cli_printf("SD card removed!");
        f_mount(NULL, "", 1);
    }
}

FRESULT log_data(char *str, UINT len){
    if (log_file_created) {
        UINT bytesWrote;
        return f_write(&logger_file, str, len, &bytesWrote);
    }
    return FR_NO_FILE;
}

void EXTI9_5_IRQHandler() {
    if (__HAL_GPIO_EXTI_GET_FLAG(SD_card_detection_Pin)) {
        if (HAL_GPIO_ReadPin(SD_card_detection_GPIO_Port, SD_card_detection_Pin) == GPIO_PIN_RESET) {
            logger_deinit();
        }
    }
    HAL_GPIO_EXTI_IRQHandler(SD_card_detection_Pin);
}