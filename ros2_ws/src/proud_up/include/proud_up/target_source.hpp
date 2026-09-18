#pragma once

// Pluggable hunter targets. No rclcpp in this header.
//
// Order of operations (node, 20 Hz):
//   1. detect_tick (cat only): YOLO on the latest BGR → CatSource::detect
//      → TargetHit with pixel + range, pose_in_base = false.
//   2. control_tick: saveli_from_tf() fills DetectInput.saveli_pose
//      (already in base_link) → SaveliSource::detect.
//   3. fill_cat_base_pose() if needed, then pick_target(), then Hunter::tick.
//
// Why a virtual base: the next plug (person, ball, …) implements
// TargetSource::detect and is listed in enabled_targets. Hunter never
// names "saveli" or "cat" — it only sees TargetHit.id.
//
// Saveli is a fiducial (AprilTag TF). Cat is COCO class 15 on the same
// yolov8n.onnx follow_the_cat already loads — not a second DNN node.

#include <memory>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "proud_up/follow_the_cat.hpp"
#include "proud_up/hunter.hpp"

namespace proud_up {

// Per-tick bag of sensor snapshots. Pointers (bgr, depth_mm) are
// non-owning — they must stay valid for the duration of detect() only.
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
  // Virtual destructor: deleting through TargetSource* calls CatSource's
  // destructor (which unloads the ONNX net). Without this, undefined.
  virtual ~TargetSource() = default;
  virtual std::string id() const = 0;
  virtual bool enabled() const = 0;
  virtual void set_enabled(bool on) = 0;
  // Timer thread only. Never call from an image / scan DDS callback.
  virtual std::optional<TargetHit> detect(const DetectInput &in) = 0;
};

// AprilTag plug. tag_id must match the printed 36h11 id (yaml: 0).
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

// YOLO plug. coco_class 15 is "cat" in the COCO column of yolov8n.onnx.
// HumanDetector is the same class follow_the_cat uses for people (class 0).
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
