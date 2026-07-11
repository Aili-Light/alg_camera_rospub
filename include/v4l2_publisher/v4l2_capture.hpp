#ifndef V4L2_PUBLISHER__V4L2_CAPTURE_HPP_
#define V4L2_PUBLISHER__V4L2_CAPTURE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace v4l2_publisher
{

struct V4L2Config
{
  std::string device_path;
  uint32_t requested_width  = 1280;
  uint32_t requested_height = 800;
  uint32_t target_width     = 640;
  uint32_t target_height    = 400;
  uint32_t pixel_format     = 0;  // V4L2_PIX_FMT_YUYV or similar, set by caller
  uint32_t buffer_count     = 4;
  double   requested_fps    = 60.0;
};

class V4L2Capture
{
public:
  explicit V4L2Capture(const V4L2Config & config);
  ~V4L2Capture();

  // Non-copyable, non-movable (owns mmap buffers and device fd)
  V4L2Capture(const V4L2Capture &) = delete;
  V4L2Capture & operator=(const V4L2Capture &) = delete;
  V4L2Capture(V4L2Capture &&) = delete;
  V4L2Capture & operator=(V4L2Capture &&) = delete;

  /// Start capture: open device, configure, mmap, stream on.
  void start();

  /// Stop capture: stream off, unmap, close.
  void stop();

  /// Block until a frame is available (2-second timeout), dequeue it,
  /// extract luminance, optionally resize. Returns the grayscale frame
  /// or std::nullopt on timeout/failure.
  /// After a successful return, frame_timestamp_ns() gives the raw
  /// V4L2 buffer timestamp in monotonic-clock nanoseconds.
  std::optional<cv::Mat> grab();

  /// Timestamp (monotonic-clock nanoseconds) of the last successfully
  /// grabbed frame, from the V4L2 buffer. Returns 0 if no frame yet.
  /// Callers apply ClockOffset::corrected_system_ns() to get system time.
  int64_t frame_timestamp_ns() const { return last_timestamp_ns_.load(); }

  // --- Query helpers ---
  uint32_t actual_width()    const { return actual_width_; }
  uint32_t actual_height()   const { return actual_height_; }
  double   actual_fps()      const { return actual_fps_; }
  uint32_t publish_width()   const { return publish_width_; }
  uint32_t publish_height()  const { return publish_height_; }
  const std::string & fourcc_string() const { return active_fourcc_string_; }

  struct Stats
  {
    uint64_t capture_count{0};
    uint64_t publish_count{0};
  };
  Stats snapshot_and_reset_stats();

private:
  void open_device();
  void configure_device();
  void init_mmap();
  void start_streaming();
  void stop_streaming();
  void unmap_buffers();
  void close_device();
  bool wait_for_frame();

  V4L2Config config_;
  int fd_{-1};
  std::vector<struct MmapBuffer> buffers_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> capture_count_{0};
  std::atomic<std::uint64_t> publish_count_{0};
  std::atomic<int64_t> last_timestamp_ns_{0};
  bool streaming_{false};
  bool uses_mplane_{false};
  uint32_t buffer_type_{0};
  uint32_t actual_width_{0};
  uint32_t actual_height_{0};
  double actual_fps_{0.0};
  uint32_t active_pixel_format_{0};
  uint32_t bytes_per_line_{0};
  std::string active_fourcc_string_{"????"};
  uint32_t publish_width_{0};
  uint32_t publish_height_{0};
  bool resize_to_target_{false};
  std::string publish_mode_{"unknown"};

  // Internal helper struct
  struct MmapBuffer
  {
    void * start{nullptr};
    size_t length{0};
  };
};

}  // namespace v4l2_publisher

#endif  // V4L2_PUBLISHER__V4L2_CAPTURE_HPP_
