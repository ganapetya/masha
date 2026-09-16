#include "proud_up/target_source.hpp"

#include <cmath>

namespace proud_up {
namespace {

double depth_at_px(const cv::Mat &depth_mm, double u, double v) {
  if (depth_mm.empty() || depth_mm.type() != CV_16UC1) {
    return 0.0;
  }
  const int x = std::max(0, std::min(depth_mm.cols - 1, static_cast<int>(std::lround(u))));
  const int y = std::max(0, std::min(depth_mm.rows - 1, static_cast<int>(std::lround(v))));
  const unsigned short mm = depth_mm.at<unsigned short>(y, x);
  if (mm < 150 || mm > 4000) {
    return 0.0;
  }
  return static_cast<double>(mm) / 1000.0;
}

}  // namespace

CatSource::CatSource(const HumanDetectConfig &cfg, std::string spoken_wav)
    : detector_(cfg), spoken_wav_(std::move(spoken_wav)) {}

std::optional<TargetHit> CatSource::detect(const DetectInput &in) {
  if (!enabled_ || in.bgr == nullptr || in.bgr->empty()) {
    return std::nullopt;
  }
  const auto det = detector_.detect(*in.bgr);
  if (!det) {
    return std::nullopt;
  }
  TargetHit hit;
  hit.id = "cat";
  hit.display_name = "cat";
  hit.spoken_wav = spoken_wav_;
  hit.pose.u = det->center.u;
  hit.pose.v = det->center.v;
  hit.pose.has_pixel = true;
  hit.pose_in_base = false;
  hit.confidence = 1.0;
  double range = 0.0;
  if (in.depth_mm != nullptr) {
    range = depth_at_px(*in.depth_mm, det->center.u, det->center.v);
  }
  if (range <= 0.0 && in.K.fy > 1.0 && det->contour.size() >= 3) {
    const double box_h = std::fabs(static_cast<double>(det->contour[2].y - det->contour[0].y));
    if (box_h > 4.0) {
      range = in.K.fy * cat_height_m_ / box_h;
      hit.pose.range_ok = false;
    }
  } else {
    hit.pose.range_ok = range > 0.0;
  }
  hit.range_m = range;
  return hit;
}

}  // namespace proud_up
