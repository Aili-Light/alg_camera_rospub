// hal_i2c.h
#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>
#include <stddef.h>

/* I2C 总线结构体 */
struct hal_i2c {
    /* 配置参数 (由平台初始化时填入) */
    const char *device_path;   // Linux: "/dev/i2c-1"
    int bus_id;                // 或者用总线号
    void *custom_config;       // 其他平台配置
    
    void *bus_handle;   // 平台相关句柄 (如 Linux fd, stm32 I2C_HandleTypeDef*)

    /* 通用传输函数 */
    int (*transfer)(struct hal_i2c *bus, uint8_t addr, uint8_t *w_buf, uint32_t w_len, uint8_t *r_buf, uint32_t r_len);
    /* 封装好的常用函数 */
    int (*read_reg8)(struct hal_i2c *bus, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    int (*read_reg16_be)(struct hal_i2c *bus, uint8_t dev_addr, uint16_t reg, uint8_t *data, uint16_t len);
    int (*write_reg8)(struct hal_i2c *bus, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);
    int (*write_reg16_be)(struct hal_i2c *bus, uint8_t dev_addr, uint16_t reg, const uint8_t *data, uint16_t len);
};

#endif