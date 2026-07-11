#ifndef I2C_DRIVE_LINUX_H
#define I2C_DRIVE_LINUX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "i2c_hal.h"

// 通用读写接口
int i2c_transformer(uint8_t i2c_bus_index, uint8_t addr, uint8_t *w_buf, uint32_t w_len, uint8_t *r_buf, uint32_t r_len);

// 0808
int i2c_write_0808(uint8_t i2c_bus_index, uint8_t addr, uint8_t reg, uint8_t dat);
int i2c_read_0808(uint8_t i2c_bus_index, uint8_t addr, uint8_t reg, uint8_t *dat);

// 1608
int i2c_write_1608(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t dat);
int i2c_read_1608(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t *dat);

// 连续读接口
int i2c_read_08XX(uint8_t i2c_bus_index, uint8_t addr, uint8_t reg, uint8_t *dat, uint32_t r_len);
int i2c_read_16XX(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t *dat, uint32_t r_len);
int i2c_xfer_transformer(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t *dat, uint32_t r_len , size_t wri_size);

// I2C硬件抽象层
int hal_i2c_linux_init(struct hal_i2c *bus, const char *device_path);

#ifdef __cplusplus
}
#endif

#endif