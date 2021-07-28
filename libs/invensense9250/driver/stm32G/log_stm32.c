/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   log_msp430.c
 *      @brief  Logging facility for the TI MSP430.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "packet.h"
#include "log.h"
#include "stm32g4xx.h"
#include "cli.h"

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (128)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

/**
 *  @brief      Prints a variable argument log message.
 *  USB output will be formatted as follows:\n
 *  packet[0]       = $\n
 *  packet[1]       = packet type (1: debug, 2: quat, 3: data)\n
 *  packet[2]       = \n for debug packets: log priority\n
 *                    for quaternion packets: unused\n
 *                    for data packets: packet content (accel, gyro, etc)\n
 *  packet[3-20]    = data\n
 *  packet[21]      = \\r\n
 *  packet[22]      = \\n
 *  @param[in]  priority    Log priority (based on Android).
 *  @param[in]  tag         File specific string.
 *  @param[in]  fmt         String of text with optional format tags.
 *
 *  @return     0 if successful.
 */
int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
    va_list args;
    char buf[BUF_SIZE];

    switch (priority) {
    case MPL_LOG_UNKNOWN:
    case MPL_LOG_DEFAULT:
    case MPL_LOG_VERBOSE:
    case MPL_LOG_DEBUG:
        cli_color_console(TEXT_GREEN_LIGHT);
        break;
    case MPL_LOG_INFO:
        cli_color_console(TEXT_BLUE_LIGHT);
        break;
    case MPL_LOG_WARN:
        cli_color_console(TEXT_YELLOW_LIGHT);
        break;
    case MPL_LOG_ERROR:
        cli_color_console(TEXT_RED_LIGHT);
        break;
    case MPL_LOG_SILENT:
        break;
    default:
        return 0;
    }

    va_start(args, fmt);
    vsnprintf(buf, BUF_SIZE, fmt, args);
    cli_printf_inline("%s", buf);
    va_end(args);

    switch (priority)
    {
    case MPL_LOG_UNKNOWN:
    case MPL_LOG_DEFAULT:
    case MPL_LOG_VERBOSE:
    case MPL_LOG_DEBUG:
        cli_color_console(TEXT_DEFAULT);
        break;
    case MPL_LOG_INFO:
        cli_color_console(TEXT_DEFAULT);
        break;
    case MPL_LOG_WARN:
        cli_color_console(TEXT_DEFAULT);
        break;
    case MPL_LOG_ERROR:
        cli_color_console(TEXT_DEFAULT);
        break;  
    case MPL_LOG_SILENT:
        break;
    }
    cli_printf("");

    return 0;
}

void eMPL_send_quat(long *quat)
{
    uint32_t div = 1<<30;
    cli_printf("quat: %7.4f %7.4f %7.4f %7.4f", quat[0] / div, quat[1] / div, quat[2] / div, quat[3] / div);
}

void eMPL_send_data(unsigned char type, long *data)
{
    char out[PACKET_LENGTH];
    int i;
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DATA;
    out[2] = type;
    switch (type) {
    /* Two bytes per-element. */
    case PACKET_DATA_ROT:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[1] >> 24);
        out[6] = (char)(data[1] >> 16);
        out[7] = (char)(data[2] >> 24);
        out[8] = (char)(data[2] >> 16);
        out[9] = (char)(data[3] >> 24);
        out[10] = (char)(data[3] >> 16);
        out[11] = (char)(data[4] >> 24);
        out[12] = (char)(data[4] >> 16);
        out[13] = (char)(data[5] >> 24);
        out[14] = (char)(data[5] >> 16);
        out[15] = (char)(data[6] >> 24);
        out[16] = (char)(data[6] >> 16);
        out[17] = (char)(data[7] >> 24);
        out[18] = (char)(data[7] >> 16);
        out[19] = (char)(data[8] >> 24);
        out[20] = (char)(data[8] >> 16);
        break;
    /* Four bytes per-element. */
    /* Four elements. */
    case PACKET_DATA_QUAT:
    /**
        out[15] = (char)(data[3] >> 24);
        out[16] = (char)(data[3] >> 16);
        out[17] = (char)(data[3] >> 8);
        out[18] = (char)data[3];
    */
        eMPL_send_quat(data);
        break;
    /* Three elements. */
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:
    /**
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        out[7] = (char)(data[1] >> 24);
        out[8] = (char)(data[1] >> 16);
        out[9] = (char)(data[1] >> 8);
        out[10] = (char)data[1];
        out[11] = (char)(data[2] >> 24);
        out[12] = (char)(data[2] >> 16);
        out[13] = (char)(data[2] >> 8);
        out[14] = (char)data[2];
    */
        cli_printf("data: %7.4f %7.4f %7.4f", data[0] / 1 << 16, data[1] / 1 << 16, data[2] / 1 << 16);
        break;
    case PACKET_DATA_HEADING:
    /**
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
    */
        cli_printf("heading: %7.4f", data[0] / 1<<16);
        break;
    default:
        return;
    }
}

/**
 * @}
**/
