#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/videoio.hpp>

namespace
{
constexpr int kWidth = 640;
constexpr int kHeight = 400;
constexpr double kRequestedFps = 60.0;

std::string fourcc_to_string(int fourcc)
{
  std::ostringstream oss;
  oss
    << static_cast<char>(fourcc & 0xFF)
    << static_cast<char>((fourcc >> 8) & 0xFF)
    << static_cast<char>((fourcc >> 16) & 0xFF)
    << static_cast<char>((fourcc >> 24) & 0xFF);
  return oss.str();
}

void print_props(cv::VideoCapture & cap)
{
  std::cout
    << " backend=" << cap.getBackendName()
    << " size=" << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT)
    << " fps=" << cap.get(cv::CAP_PROP_FPS)
    << " fourcc=" << fourcc_to_string(static_cast<int>(cap.get(cv::CAP_PROP_FOURCC)))
    << std::endl;
}

void test_open(const std::string & label, const std::string & dev_path, int dev_index, int api_preference, bool use_path)
{
  cv::VideoCapture cap;
  bool opened = false;

  if (use_path) {
    std::cout << "open " << label << " path=" << dev_path << " api=" << api_preference;
    opened = cap.open(dev_path, api_preference);
  } else {
    std::cout << "open " << label << " index=" << dev_index << " api=" << api_preference;
    opened = cap.open(dev_index, api_preference);
  }

  std::cout << " opened=" << (opened ? "true" : "false");
  if (!opened) {
    std::cout << std::endl;
    return;
  }

  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
  cap.set(cv::CAP_PROP_FRAME_WIDTH, kWidth);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, kHeight);
  cap.set(cv::CAP_PROP_FPS, kRequestedFps);
  cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
  print_props(cap);

  cv::Mat frame;
  bool read_ok = cap.read(frame);
  std::cout << "first_read ok=" << (read_ok ? "true" : "false");
  if (read_ok) {
    std::cout << " frame=" << frame.cols << "x" << frame.rows << " channels=" << frame.channels();
  }
  std::cout << std::endl;

  cap.release();
}
}  // namespace

int main()
{
  const std::vector<std::pair<std::string, int>> devices = {
    {"/dev/video0", 0},
    {"/dev/video1", 1},
  };
  const std::vector<std::pair<std::string, int>> apis = {
    {"CAP_V4L2", cv::CAP_V4L2},
    {"CAP_ANY", cv::CAP_ANY},
    {"CAP_GSTREAMER", cv::CAP_GSTREAMER},
  };

  for (const auto & [dev_path, dev_index] : devices) {
    std::cout << "=== device " << dev_path << " exists=" << (std::filesystem::exists(dev_path) ? "true" : "false") << " ===" << std::endl;
    for (const auto & [api_name, api_value] : apis) {
      test_open(api_name + " path", dev_path, dev_index, api_value, true);
      test_open(api_name + " index", dev_path, dev_index, api_value, false);
    }
    std::cout << std::endl;
  }

  return 0;
}
