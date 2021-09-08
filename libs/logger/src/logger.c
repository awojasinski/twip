#include "stm32g4xx_hal.h"
#include "main.h"

#include "string.h"

#include "cli.h"
#include "logger.h"

#define __LOG_FILENAME "log.bin"

//static log_t *logger_read(void);

FATFS FileSystem;
FIL logger_file;

volatile bool logger_initialized = false;
log_t log_buffer[LOG_FIFO_SIZE];
log_t *p_log_buffer[LOG_FIFO_SIZE];
static volatile log_fifo_t m_logger_fifo =
    {.tail = 0,
     .head = 0,
     .size = LOG_FIFO_SIZE,
     .data = p_log_buffer};

void logger_init()
{
    if (HAL_GPIO_ReadPin(SD_card_detection_GPIO_Port, SD_card_detection_Pin) == GPIO_PIN_SET)
    {
        if (f_mount((FATFS *)&FileSystem, "", 1) == FR_OK)
        {
            cli_printf("SD card detected!");
            if (f_open(&logger_file, __LOG_FILENAME, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK)
            {
                logger_initialized = true;
            }
        }
    }
}

void logger_deinit()
{
    if (logger_initialized)
    {
        if (f_close(&logger_file) == FR_OK)
        {
            logger_initialized = false;
        };
        cli_printf("SD card removed!");
        f_mount(NULL, "", 1);
    }
}

void logger_main()
{
    static bool sync = false;
    log_t *p_log = logger_read();

    if (p_log != NULL && logger_initialized)
    {
        f_write(&logger_file, p_log, sizeof(log_t), NULL);
        if (sync)
        {
            f_sync(&logger_file);
        }
        sync = !sync;
    }
}

int8_t logger_write(log_t *p_log)
{
    if ((m_logger_fifo.head + 1) % m_logger_fifo.size == m_logger_fifo.tail)
    {
        return -1;
    }
    memcpy(&log_buffer[m_logger_fifo.head], p_log, sizeof(log_t));
    m_logger_fifo.data[m_logger_fifo.head] = &log_buffer[m_logger_fifo.head];
    m_logger_fifo.head = (m_logger_fifo.head + 1) % m_logger_fifo.size;
    return 0;
}

log_t *logger_read()
{
    if (m_logger_fifo.tail == m_logger_fifo.head)
    {
        return NULL;
    }
    log_t *p_handle = m_logger_fifo.data[m_logger_fifo.tail];
    m_logger_fifo.data[m_logger_fifo.tail] = NULL;
    m_logger_fifo.tail = (m_logger_fifo.tail + 1) % m_logger_fifo.size;

    return p_handle;
}

void fatfs_test()
{
    FATFS FS;
    FIL f;
    FRESULT res;

    res = f_mount(&FS, "", 1); //1=mount now
    if (res != FR_OK)
    {
        cli_printf("f_mount error (%i)\r\n", res);
        while (1)
            ;
    }

    //Let's get some statistics from the SD card
    DWORD free_clusters, free_sectors, total_sectors;

    FATFS *getFreeFs;

    res = f_getfree("", &free_clusters, &getFreeFs);
    if (res != FR_OK)
    {
        cli_printf("f_getfree error (%i)\r\n", res);
        while (1)
            ;
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    cli_printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    //Now let's try to open fe "test.txt"
    res = f_open(&f, "test.txt", FA_READ);
    if (res != FR_OK)
    {
        cli_printf("f_open error (%i)\r\n");
        while (1)
            ;
    }
    cli_printf("I was able to open 'test.txt' for reading!\r\n");

    //Read 30 bytes from "test.txt" on the SD card
    BYTE readBuf[30];

    //We can either use f_read OR f_gets to get data out of fes
    //f_gets is a wrapper on f_read that does some string formatting for us
    TCHAR *rres = f_gets((TCHAR *)readBuf, 30, &f);
    if (rres != 0)
    {
        cli_printf("Read string from 'test.txt' contents: %s\r\n", readBuf);
    }
    else
    {
        cli_printf("f_gets error (%i)\r\n", res);
    }

    //Be a tidy kiwi - don't forget to close your fe!
    f_close(&f);

    //Now let's try and write a fe "write.txt"
    res = f_open(&f, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if (res == FR_OK)
    {
        cli_printf("I was able to open 'write.txt' for writing\r\n");
    }
    else
    {
        cli_printf("f_open error (%i)\r\n", res);
    }

    //Copy in a string
    strncpy((char *)readBuf, "a new file is made!", 19);
    UINT bytesWrote;
    res = f_write(&f, readBuf, 19, &bytesWrote);
    if (res == FR_OK)
    {
        cli_printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
    }
    else
    {
        cli_printf("f_write error (%i)\r\n");
    }

    //Be a tidy kiwi - don't forget to close your fe!
    f_close(&f);

    //We're done, so de-mount the drive
    f_mount(NULL, "", 1);
}

void EXTI9_5_IRQHandler()
{
    if (__HAL_GPIO_EXTI_GET_FLAG(SD_card_detection_Pin))
    {
        if (HAL_GPIO_ReadPin(SD_card_detection_GPIO_Port, SD_card_detection_Pin) == GPIO_PIN_RESET)
        {
            logger_deinit();
        }
        else
        {
            logger_init();
        }
    }
    HAL_GPIO_EXTI_IRQHandler(SD_card_detection_Pin);
}
