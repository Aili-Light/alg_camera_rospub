#ifndef V4L2_PUBLISHER__IMU_DEVICE_HPP_
#define V4L2_PUBLISHER__IMU_DEVICE_HPP_

namespace v4l2_publisher
{

/// Abstract IMU device interface.
/// New IMU chips are supported by implementing this interface.
class ImuDevice
{
public:
  virtual ~ImuDevice() = default;

  /// Read accelerometer data in g.
  /// Returns true on success.
  virtual bool read_accel(float & x, float & y, float & z) = 0;

  /// Read gyroscope data in degrees per second.
  /// Returns true on success.
  virtual bool read_gyro(float & x, float & y, float & z) = 0;

  /// Read temperature in degrees Celsius.
  /// Returns true on success.
  virtual bool read_temp(float & temp) = 0;
};

}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__IMU_DEVICE_HPP_
