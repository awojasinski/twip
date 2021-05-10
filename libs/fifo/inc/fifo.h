#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

typedef struct {
  uint8_t *buffer;
  uint16_t size;
  uint16_t head;
  uint16_t tail;
} fifo_t;

void fifo_init(fifo_t*, uint8_t*, uint16_t);
uint16_t fifo_data_size(fifo_t*);
uint16_t fifo_write(fifo_t*, const char*, uint16_t);
uint16_t fifo_read(fifo_t*, char*, uint16_t);

#endif