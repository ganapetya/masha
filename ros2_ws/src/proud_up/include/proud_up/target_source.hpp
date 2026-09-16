#pragma once

// Pluggable hunter targets. No rclcpp in this header.
//
// The node copies the latest image / TF pose on a timer (never on the
// DDS callback), then each enabled TargetSource::detect() fills a
// TargetHit. Saveli is a fiducial (pose already in base_link). Cat is
// COCO class 15 on the existing yolov8n.onnx — not a second DNN node.

#include <memory>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "proud_up/follow_the_cat.hpp"
#include "proud_up/hunter.hpp"

namespace proud_up {

struct DetectInput {
  bool have_saveli_pose{false};
  int tag_id{0};
  PoseInBase saveli_pose;
  const cv::Mat *bgr{nullptr};
  const cv::Mat *depth_mm{nullptr};  // 16UC1 millimetres, or null
  CameraIntrinsics K{};
  double now_s{0.0};
};

class TargetSource {
 public:
  virtual ~TargetSource() = default;
  virtual std::string id() const = 0;
  virtual bool enabled() const = 0;
  virtual void set_enabled(bool on) = 0;
  // Timer thread only. Never call from an image / scan DDS callback.
  virtual std::optional<TargetHit> detect(const DetectInput &in) = 0;
};

class SaveliSource : public TargetSource {
 public:
  explicit SaveliSource(int tag_id = 0, std::string spoken_wav = "saveli.wav");
  std::string id() const override { return "saveli"; }
  bool enabled() const override { return enabled_; }
  void set_enabled(bool on) override { enabled_ = on; }
  std::optional<TargetHit> detect(const DetectInput &in) override;

 private:
  int tag_id_{0};
  std::string spoken_wav_;
  bool enabled_{true};
};

class CatSource : public TargetSource {
 public:
  explicit CatSource(const HumanDetectConfig &cfg, std::string spoken_wav = "cat.wav");
  std::string id() const override { return "cat"; }
  bool enabled() const override { return enabled_; }
  void set_enabled(bool on) override { enabled_ = on; }
  bool net_ok() const { return detector_.person_ok(); }
  std::optional<TargetHit> detect(const DetectInput &in) override;

 private:
  HumanDetector detector_;
  std::string spoken_wav_;
  bool enabled_{false};
  double cat_height_m_{0.25};  // bbox-height heuristic if depth is missing
};

}  // namespace proud_up
