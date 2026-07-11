#ifndef ALG_CAMERA_ROSPUB__IMU_ICM_45686_HPP_
#define ALG_CAMERA_ROSPUB__IMU_ICM_45686_HPP_

#include <cstdint>
#include <string>

#include "alg_camera_rospub/imu_device.hpp"

// Forward declarations for C types
struct hal_i2c;
struct imu_device;

namespace alg_camera_rospub
{

/// ICM-45686 IMU driver.
/// Wraps the C HAL (i2c_drive_linux) and chip-specific register logic.
class ImuIcm45686 : public ImuDevice
{
public:
  /// Construct and initialize the IMU.
  /// @param i2c_device_path  e.g. "/dev/i2c-3"
  /// @param i2c_addr         7-bit I2C address (e.g. 0x69)
  /// @throws std::runtime_error if I2C init or chip init fails
  ImuIcm45686(const std::string & i2c_device_path, uint8_t i2c_addr);
  ~ImuIcm45686() override;

  ImuIcm45686(const ImuIcm45686 &) = delete;
  ImuIcm45686 & operator=(const ImuIcm45686 &) = delete;
  ImuIcm45686(ImuIcm45686 &&) = delete;
  ImuIcm45686 & operator=(ImuIcm45686 &&) = delete;

  bool read_accel(float & x, float & y, float & z) override;
  bool read_gyro(float & x, float & y, float & z) override;
  bool read_temp(float & temp) override;

private:
  struct hal_i2c * i2c_bus_{nullptr};
  struct imu_device * dev_{nullptr};
};

}  // namespace alg_camera_rospub

#endif  // ALG_CAMERA_ROSPUB__IMU_ICM_45686_HPP_
