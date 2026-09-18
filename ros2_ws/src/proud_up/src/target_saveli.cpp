#include "proud_up/target_source.hpp"

namespace proud_up {

SaveliSource::SaveliSource(int tag_id, std::string spoken_wav)
    : tag_id_(tag_id), spoken_wav_(std::move(spoken_wav)) {}

// The node already looked up T_base_tag. This plug only packages that
// pose into a TargetHit. No OpenCV, no TF — so gtest can call it with
// a fake DetectInput and never spin a node.
//
// std::move on spoken_wav in the ctor: we steal the string instead of
// copying. After the caller's temporary dies, we still own the path.
std::optional<TargetHit> SaveliSource::detect(const DetectInput &in) {
  if (!enabled_ || !in.have_saveli_pose) {
    return std::nullopt;
  }
  if (in.tag_id != tag_id_) {
    return std::nullopt;
  }
  TargetHit hit;
  hit.id = "saveli";
  hit.display_name = "Saveli";
  hit.spoken_wav = spoken_wav_;
  hit.pose = in.saveli_pose;
  hit.pose.range_ok = true;
  hit.pose_in_base = true;
  hit.range_m = std::hypot(in.saveli_pose.x, in.saveli_pose.y);
  hit.confidence = 1.0;
  return hit;
}

}  // namespace proud_up
