#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace proud_up {

// Encoded snapshot. Capture produces unique ownership; disk + SMTP share it.
struct CameraFrame {
  int width{0};
  int height{0};
  std::string encoding{"bgr8"};
  std::vector<std::uint8_t> jpeg;
  std::string captured_at_utc;
};

// Exclusive ownership of the JPEG buffer. Caller moves it into shared_ptr
// only when more than one consumer needs the same bytes.
std::unique_ptr<CameraFrame> encode_jpeg(const cv::Mat &bgr, int quality);

bool save_jpeg(const std::shared_ptr<const CameraFrame> &frame,
               const std::string &path);

}  // namespace proud_up
