// imu_core.h
#ifndef IMU_CORE_H
#define IMU_CORE_H

#include "i2c_hal.h"
#include <stdint.h>

struct imu_device;

/* IMU 虚函数表 */
struct imu_ops {
    int (*init)(struct imu_device *dev);
    int (*read_accel)(struct imu_device *dev, float *x, float *y, float *z);
    int (*read_gyro)(struct imu_device *dev, float *x, float *y, float *z);
    int (*read_temp)(struct imu_device *dev, float *temp);
    int (*deinit)(struct imu_device *dev);
};

/* IMU 基类 (抽象) */
struct imu_device {
    const struct imu_ops *ops;   // vtable
    struct hal_i2c *i2c_bus;     // 当前使用的 I2C 总线
    uint8_t i2c_addr;            // 设备地址
    void *priv;                  // 派生类私有数据 (可选)
};



/* 通用辅助函数 (如果希望对外提供统一接口，可以再加一层 wrapper) */
static inline int imu_init(struct imu_device *dev) {
    return dev->ops->init ? dev->ops->init(dev) : -1;
}

static inline int imu_read_accel(struct imu_device *dev, float *x, float *y, float *z) {
    return dev->ops->read_accel(dev, x, y, z);
}
static inline int imu_read_gyro(struct imu_device *dev, float *x, float *y, float *z) {
    return dev->ops->read_gyro(dev, x, y, z);
}

static inline int imu_read_temp(struct imu_device *dev,  float *temp) {
    return dev->ops->read_temp(dev, temp);
}

#endif


