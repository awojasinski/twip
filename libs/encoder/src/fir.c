#include "fir.h"

void fir_ma_init(fir_ma_filter_t *p_filter, int32_t *p_buffer, uint8_t size)
{
    p_filter->buffer = p_buffer;
    p_filter->size = size;
}

void fir_ma_update(fir_ma_filter_t *p_filter, int32_t data)
{
    if (p_filter->fill)
    {
        p_filter->sum -= p_filter->buffer[p_filter->idx];
    }
    p_filter->buffer[p_filter->idx] = data;
    p_filter->sum += data;
    p_filter->idx = (uint8_t)((p_filter->idx + 1) % p_filter->size);
    if (p_filter->idx == 0 && !p_filter->fill)
    {
        p_filter->fill = true;
    }
}

int32_t fir_ma_read(fir_ma_filter_t *p_filter)
{
    return p_filter->sum / p_filter->size;
}
