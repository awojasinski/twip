#include "fifo.h"

void fifo_init(fifo_t *fifo, uint8_t *buffer, uint16_t size) {
  fifo->buffer = buffer;
  fifo->size = size;
  fifo->head = 0;
  fifo->tail = 0;
}

uint16_t fifo_data_size(fifo_t *fifo) {
  if (fifo->head != fifo->tail) {
    if (fifo->head > fifo->tail) {
      return (uint16_t)(fifo->head - fifo->tail);
    } else {
      return (uint16_t)(fifo->size - fifo->tail + fifo->head);
    }
  } else {
    return 0;
  }
}

uint16_t fifo_write(fifo_t *fifo, const char *str, uint16_t size) {
  for (uint16_t i=0; i<size; i++) {
    if (fifo->head + 1 == fifo->tail || (fifo->tail == 0 && fifo->head + 1 == fifo->size)) {
      return i;
    } else {
      fifo->buffer[fifo->head] = str[i];
      fifo->head++;
      if(fifo->head == fifo->size) {
        fifo->head = 0;
      }
    }
  }
  return size;
}

uint16_t fifo_read(fifo_t *fifo, char *str, uint16_t size) {
  for (uint16_t i=0; i<size; i++) {
    if (fifo->tail != fifo->head) {
      str[i] = fifo->buffer[fifo->tail];
      fifo->tail++;
      if (fifo->tail == fifo->size) {
        fifo->tail = 0;
      }
    } else {
      return i;
    }
  }
  return size;
}