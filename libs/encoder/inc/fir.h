#ifndef FIR_H
#define FIR_H

#include "stm32g4xx_hal.h"
#include "stdbool.h"

typedef struct
{
    int32_t *buffer;
    int32_t sum;
    uint8_t size;
    uint8_t idx;
    bool fill;

} fir_ma_filter_t;

void fir_ma_init(fir_ma_filter_t *, int32_t *, uint8_t);
void fir_ma_update(fir_ma_filter_t *, int32_t);
int32_t fir_ma_read(fir_ma_filter_t *);

#endif
