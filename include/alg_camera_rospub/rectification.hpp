#ifndef ALG_CAMERA_ROSPUB__RECTIFICATION_HPP_
#define ALG_CAMERA_ROSPUB__RECTIFICATION_HPP_

#include <string>

#include <opencv2/core.hpp>

namespace alg_camera_rospub
{

struct RectificationMaps
{
  cv::Mat map1;
  cv::Mat map2;
};

/// Load rectification maps from an OpenCV YAML calibration file.
/// prefix is "left" or "right" to match map1_left/map2_left or map1_right/map2_right.
RectificationMaps load_rectification_maps(const std::string & yaml_path,
                                          const std::string & prefix);

/// Validate maps for a given output size. Throws std::runtime_error on mismatch.
void validate_rectification_maps(const RectificationMaps & maps,
                                 const std::string & name,
                                 uint32_t width,
                                 uint32_t height);

}  // namespace alg_camera_rospub

#endif  // ALG_CAMERA_ROSPUB__RECTIFICATION_HPP_
