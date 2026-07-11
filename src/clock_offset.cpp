#include "alg_camera_rospub/clock_offset.hpp"

#include <chrono>

namespace alg_camera_rospub
{

ClockOffset::ClockOffset()
: offset_ns_(0)
{
  calibrate();
}

void ClockOffset::calibrate()
{
  const auto system_now = std::chrono::system_clock::now();
  const auto steady_now = std::chrono::steady_clock::now();

  const auto system_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    system_now.time_since_epoch()).count();
  const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();

  offset_ns_ = system_ns - steady_ns;
}

int64_t ClockOffset::corrected_system_ns(int64_t steady_ns) const
{
  return steady_ns + offset_ns_;
}

int64_t ClockOffset::now_ns() const
{
  const auto steady_now = std::chrono::steady_clock::now();
  const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();
  return corrected_system_ns(steady_ns);
}

}  // namespace alg_camera_rospub
