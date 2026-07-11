#include "alg_camera_rospub/imu_icm_45686.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include <unistd.h>

extern "C" {
#include "i2c_hal.h"
#include "i2c_drive_linux.h"
#include "imu_core.h"
}

// ==========================================================================
// ICM-45686 register definitions
// ==========================================================================
#define IMU_ICM_45686_PWR_MGMT0         0x10
#define IMU_ICM_45686_ACCEL_CONFIG0     0x1B
#define IMU_ICM_45686_GYRO_CONFIG0      0x1C
#define IMU_ICM_45686_ACCEL_DATA_X1_UI  0x00
#define IMU_ICM_45686_GYRO_DATA_X1_UI   0x06
#define IMU_ICM_45686_TEMP_DATA1_UI     0x0C

// ==========================================================================
// C HAL: ICM-45686 ops (implements imu_ops vtable)
// ==========================================================================

static int icm45686_i2c_read_regs(struct imu_device * dev, uint8_t reg, uint8_t * buf, uint16_t len)
{
  return dev->i2c_bus->transfer(dev->i2c_bus, dev->i2c_addr, &reg, 1, buf, len);
}

static int icm45686_i2c_write_reg(struct imu_device * dev, uint8_t reg, uint8_t val)
{
  uint8_t buf_w[2] = {reg, val};
  return dev->i2c_bus->transfer(dev->i2c_bus, dev->i2c_addr, buf_w, 2, nullptr, 0);
}

static int icm45686_init(struct imu_device * dev)
{
  // Reset the IMU
  uint8_t val = 0x01;                // DEVICE_RESET bit
  if (icm45686_i2c_write_reg(dev, IMU_ICM_45686_PWR_MGMT0, val) < 0) {
    return -1;
  }
  // Wait for reset to complete (approx 1 ms)
  usleep(2000);

  // Set power management: enable all sensors in low-noise mode
  val = 0x0F;                        // accel LN + gyro LN + temp enable
  if (icm45686_i2c_write_reg(dev, IMU_ICM_45686_PWR_MGMT0, val) < 0) {
    return -1;
  }

  // Configure accelerometer: +/-8g (sensitivity = 4096 LSB/g)
  val = 0x01;                        // ACCEL_UI_FS_SEL = 1 (+/-8g)
  if (icm45686_i2c_write_reg(dev, IMU_ICM_45686_ACCEL_CONFIG0, val) < 0) {
    return -1;
  }

  // Configure gyroscope: 2000 dps (sensitivity = 16.4 LSB/(dps))
  val = 0x00;                        // GYRO_UI_FS_SEL = 0 (2000 dps)
  if (icm45686_i2c_write_reg(dev, IMU_ICM_45686_GYRO_CONFIG0, val) < 0) {
    return -1;
  }

  return 0;
}

static int icm45686_read_accel(struct imu_device * dev, float * x, float * y, float * z)
{
  uint8_t buf[6];
  if (icm45686_i2c_read_regs(dev, IMU_ICM_45686_ACCEL_DATA_X1_UI, buf, 6) < 0) {
    return -1;
  }

  int16_t raw_x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  int16_t raw_y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
  int16_t raw_z = static_cast<int16_t>((buf[5] << 8) | buf[4]);

  *x = raw_x / 4096.0f;    // +/-8g scale factor
  *y = raw_y / 4096.0f;
  *z = raw_z / 4096.0f;
  return 0;
}

static int icm45686_read_gyro(struct imu_device * dev, float * x, float * y, float * z)
{
  uint8_t buf[6];
  if (icm45686_i2c_read_regs(dev, IMU_ICM_45686_GYRO_DATA_X1_UI, buf, 6) < 0) {
    return -1;
  }

  int16_t raw_x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  int16_t raw_y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
  int16_t raw_z = static_cast<int16_t>((buf[5] << 8) | buf[4]);

  *x = raw_x / 16.4f;     // 2000 dps scale factor
  *y = raw_y / 16.4f;
  *z = raw_z / 16.4f;
  return 0;
}

static int icm45686_read_temp(struct imu_device * dev, float * temp)
{
  uint8_t buf[2];
  if (icm45686_i2c_read_regs(dev, IMU_ICM_45686_TEMP_DATA1_UI, buf, 2) < 0) {
    return -1;
  }

  int16_t raw_temp = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  *temp = (raw_temp / 128.0f) + 25.0f;
  return 0;
}

static int icm45686_deinit(struct imu_device * dev)
{
  // Put the IMU into sleep mode
  icm45686_i2c_write_reg(dev, IMU_ICM_45686_PWR_MGMT0, 0x00);
  return 0;
}

static const struct imu_ops icm45686_ops = {
  icm45686_init,
  icm45686_read_accel,
  icm45686_read_gyro,
  icm45686_read_temp,
  icm45686_deinit,
};

struct imu_device * imu_icm_45686_create(struct hal_i2c * bus, uint8_t addr)
{
  struct imu_device * dev = static_cast<struct imu_device *>(calloc(1, sizeof(struct imu_device)));
  if (!dev) {
    return nullptr;
  }

  dev->ops = &icm45686_ops;
  dev->i2c_bus = bus;
  dev->i2c_addr = addr;
  dev->priv = nullptr;

  if (imu_init(dev) < 0) {
    free(dev);
    return nullptr;
  }

  return dev;
}

void imu_icm_45686_destroy(struct imu_device * dev)
{
  if (dev) {
    icm45686_deinit(dev);
    free(dev);
  }
}

// ==========================================================================
// C++ Wrapper: ImuIcm45686
// ==========================================================================

namespace alg_camera_rospub
{

ImuIcm45686::ImuIcm45686(const std::string & i2c_device_path, uint8_t i2c_addr)
{
  i2c_bus_ = new struct hal_i2c;
  std::memset(i2c_bus_, 0, sizeof(*i2c_bus_));

  // Initialize the I2C bus (file descriptor kept open for lifetime)
  if (hal_i2c_linux_init(i2c_bus_, i2c_device_path.c_str()) < 0) {
    delete i2c_bus_;
    i2c_bus_ = nullptr;
    throw std::runtime_error("Failed to init I2C bus on " + i2c_device_path);
  }

  // Create and initialize the IMU device
  dev_ = imu_icm_45686_create(i2c_bus_, static_cast<uint8_t>(i2c_addr));
  if (!dev_) {
    delete i2c_bus_;
    i2c_bus_ = nullptr;
    throw std::runtime_error(
      "Failed to create IMU ICM-45686 at address 0x" +
      std::to_string(i2c_addr));
  }

  fprintf(stderr, "[ImuIcm45686] Initialized on %s at 0x%02X\n",
          i2c_device_path.c_str(), i2c_addr);
}

ImuIcm45686::~ImuIcm45686()
{
  if (dev_) {
    imu_icm_45686_destroy(dev_);
    dev_ = nullptr;
  }
  // The C HAL does not expose a close function for the I2C bus handle;
  // the fd is closed when the process exits.
  delete i2c_bus_;
}

bool ImuIcm45686::read_accel(float & x, float & y, float & z)
{
  return imu_read_accel(dev_, &x, &y, &z) == 0;
}

bool ImuIcm45686::read_gyro(float & x, float & y, float & z)
{
  return imu_read_gyro(dev_, &x, &y, &z) == 0;
}

bool ImuIcm45686::read_temp(float & temp)
{
  return imu_read_temp(dev_, &temp) == 0;
}

}  // namespace alg_camera_rospub
