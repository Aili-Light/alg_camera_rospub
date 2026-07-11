#include "v4l2_publisher/rectification.hpp"

#include <stdexcept>

#include <opencv2/core/persistence.hpp>

namespace v4l2_publisher
{

RectificationMaps load_rectification_maps(const std::string & yaml_path,
                                          const std::string & prefix)
{
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to open calibration file: " + yaml_path);
  }

  const std::string key_map1 = "map1_" + prefix;
  const std::string key_map2 = "map2_" + prefix;

  RectificationMaps maps;
  fs[key_map1] >> maps.map1;
  fs[key_map2] >> maps.map2;

  if (maps.map1.empty() || maps.map2.empty()) {
    throw std::runtime_error(
      "Missing rectification map(s) " + key_map1 + "/" + key_map2 +
      " in " + yaml_path);
  }
  if (maps.map1.size() != maps.map2.size()) {
    throw std::runtime_error(
      "Rectification maps for '" + prefix + "' have mismatched sizes");
  }
  if (maps.map1.type() != CV_32FC1 || maps.map2.type() != CV_32FC1) {
    throw std::runtime_error(
      "Rectification maps for '" + prefix + "' must be CV_32FC1");
  }

  return maps;
}

void validate_rectification_maps(const RectificationMaps & maps,
                                 const std::string & name,
                                 uint32_t width,
                                 uint32_t height)
{
  const cv::Size expected(static_cast<int>(width), static_cast<int>(height));
  if (maps.map1.size() != expected || maps.map2.size() != expected) {
    throw std::runtime_error(
      "Rectification maps for '" + name + "' have size " +
      std::to_string(maps.map1.cols) + "x" + std::to_string(maps.map1.rows) +
      " but publish output is " + std::to_string(width) + "x" + std::to_string(height));
  }
}

}  // namespace v4l2_publisher
