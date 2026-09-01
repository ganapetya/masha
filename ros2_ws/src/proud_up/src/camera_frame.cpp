#include "proud_up/camera_frame.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

namespace proud_up {
namespace {

std::string utc_now() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

}  // namespace

std::unique_ptr<CameraFrame> encode_jpeg(const cv::Mat &bgr, int quality) {
  if (bgr.empty()) {
    return nullptr;
  }

  std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality};
  std::vector<std::uint8_t> jpeg;
  if (!cv::imencode(".jpg", bgr, jpeg, params) || jpeg.empty()) {
    return nullptr;
  }

  auto frame = std::make_unique<CameraFrame>();
  frame->width = bgr.cols;
  frame->height = bgr.rows;
  frame->encoding = "bgr8";
  frame->jpeg = std::move(jpeg);
  frame->captured_at_utc = utc_now();
  return frame;
}

bool save_jpeg(const std::shared_ptr<const CameraFrame> &frame, const std::string &path) {
  if (!frame || frame->jpeg.empty() || path.empty()) {
    return false;
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }
  out.write(reinterpret_cast<const char *>(frame->jpeg.data()),
            static_cast<std::streamsize>(frame->jpeg.size()));
  return static_cast<bool>(out);
}

}  // namespace proud_up
