
#ifndef PORTME_H
#define PORTME_H

#include "i2c.h"
#include "main.h"

#define usleep(x) HAL_Delay((uint32_t)(x / 1000))
#define i2c_write_bytes(slv_addr, addr, len, p_data) HAL_I2C_Mem_Write(&hi2c1, (uint16_t)slv_addr << 1, addr, 1, p_data, len, 100)

int i2c_read_byte(int slv_addr, uint8_t addr, uint8_t *p_data);
int i2c_read_bytes(int slv_addr, uint8_t addr, uint16_t len, uint8_t *p_data);
int i2c_read_word(int slv_addr, uint8_t addr, uint16_t *p_data);
int i2c_write_byte(int slv_addr, uint8_t addr, uint8_t data);

#endif
