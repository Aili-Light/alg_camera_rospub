#ifndef ALG_CAMERA_ROSPUB__CLOCK_OFFSET_HPP_
#define ALG_CAMERA_ROSPUB__CLOCK_OFFSET_HPP_

#include <cstdint>

namespace alg_camera_rospub
{

class ClockOffset
{
public:
  ClockOffset();

  /// Re-measure the offset between system_clock and steady_clock.
  /// Call at init and periodically to guard against clock drift.
  void calibrate();

  /// Convert a monotonic (steady_clock) timestamp in nanoseconds
  /// to an offset-corrected system-clock timestamp in nanoseconds.
  int64_t corrected_system_ns(int64_t steady_ns) const;

  /// Current offset-corrected system time in nanoseconds.
  /// Equivalent to corrected_system_ns(steady_clock::now()).
  int64_t now_ns() const;

private:
  int64_t offset_ns_;
};

}  // namespace alg_camera_rospub

#endif  // ALG_CAMERA_ROSPUB__CLOCK_OFFSET_HPP_
