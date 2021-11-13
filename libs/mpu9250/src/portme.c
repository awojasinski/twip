#include "portme.h"

int i2c_read_byte(int slv_addr, uint8_t addr, uint8_t *p_data)
{
    return i2c_read_bytes(slv_addr, addr, 1, p_data);
}

int i2c_read_bytes(int slv_addr, uint8_t addr, uint16_t len, uint8_t *p_data)
{
    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)slv_addr << 1, addr, 1, p_data, len, 100);
    return len;
}

int i2c_read_word(int slv_addr, uint8_t addr, uint16_t *p_data)
{
    uint8_t data[2];
    int ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)slv_addr << 1, addr, 1, data, 2, 100);
    *p_data = data[0] << 8 | data[1];
    return -1 * ret;
}

int i2c_write_byte(int slv_addr, uint8_t addr, uint8_t data)
{
    uint8_t p_data = data;
    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)slv_addr << 1, addr, 1, &p_data, 1, 100);
}
