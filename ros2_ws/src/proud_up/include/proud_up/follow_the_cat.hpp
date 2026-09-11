#pragma once

// Follow-the-cat library: find a person in a camera image and turn that
// pixel into pan/tilt servo pulses.
//
// This header has no ROS. The node (follow_the_cat_node.cpp) calls these
// functions from a timer after it has copied the latest image pointer.
// Tests call the same functions with fake images — no robot required.
//
// Pipeline a learner can follow top to bottom:
//
//   image  →  YOLO person box  →  aim pixel (torso, not the top of the box)
//         →  ray through the camera lens
//         →  yaw / pitch (how far the person is from the image centre)
//         →  a small step on arm servos 19 (pan) and 22 (tilt)
//
// Optical frame (ROS camera convention, not the robot base):
//   X right, Y down, Z forward (out of the lens).
// A pixel (u, v) is not a 3D point in the room. It is a direction. This
// week we have no depth, so we never compute "metres to the person".

#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>

namespace proud_up {

// ---------------------------------------------------------------------------
// 1. Image plane — where the person appears in the picture
// ---------------------------------------------------------------------------

// Pixel coordinates in the Aurora RGB image. The origin is the top-left
// corner of the frame. u grows to the right, v grows downward. Units are
// pixels, not metres.
struct Pixel {
  double u{0.0};
  double v{0.0};
};

// One detection the rest of the node can use. Both YOLO (a person) and LBP
// (a face) fill this same struct. After detection, the control loop does
// not care which detector produced the box — it only needs a pixel to look
// at, a rectangle to draw, and a short label.
//
// std::optional<PersonDetection> is the "did we see anyone?" result:
//   has a value  →  there is a box this frame (or a remembered coast box)
//   empty        →  a miss: Idle waits, Tracking may go Lost, Moving aborts
struct PersonDetection {
  Pixel center;                       // the pixel we aim the camera at
  std::vector<cv::Point> contour;     // rectangle corners, for the debug image
  std::vector<cv::Point> approx;      // same as contour for a YOLO / face box
  double area{0.0};                   // box width * height, in px^2
  double aspect{0.0};                 // width / height; a standing person is < 1
  const char *label{"person"};        // "person", "face", or "coast"
};

// ---------------------------------------------------------------------------
// 2. Finding a person — YOLOv8n, with an optional face inside the box
// ---------------------------------------------------------------------------

// YOLOv8n (COCO class 0 = person) finds a person from many viewpoints:
// standing, sitting, facing away, across the room. The LBP face cascade is
// a second, cheaper pass. It only helps when someone is looking at the
// lens; it is optional and off by default because it can lock onto a
// window or a curtain instead of a face.
//
// person_head_frac chooses the aim pixel inside the YOLO box. 0.0 is the
// top edge, 0.5 is the middle, 1.0 is the bottom. 0.45 aims at the torso.
// Aiming too high (for example 0.12) sends the camera up toward the
// ceiling and it stays there.
struct HumanDetectConfig {
  std::string cascade_dir{"/usr/share/opencv4/haarcascades"};
  std::string lbp_dir{"/usr/share/opencv4/lbpcascades"};
  std::string person_onnx;  // empty = skip YOLO (tests, or face-only)
  double image_scale{0.45};  // LBP face runs on a smaller grey image
  double scale_factor{1.2};
  int min_neighbors{3};
  int min_face{24};  // pixels in the original image, not the scaled copy
  double person_conf{0.45};
  double person_iou{0.45};
  int person_imgsz{320};
  bool dnn_cuda{true};
  bool enable_face_refine{false};
  double person_head_frac{0.45};
  double max_box_frac{0.40};     // a box that fills the frame is a wall, not a person
  double max_aspect{1.05};       // a person is taller than they are wide
  double track_gate_frac{0.22};  // stay on the same person; do not jump to a neighbour
};

class HumanDetector {
 public:
  explicit HumanDetector(const HumanDetectConfig &cfg = {});
  ~HumanDetector();
  bool ok() const { return ok_ || person_ok_; }
  bool person_ok() const { return person_ok_; }
  bool using_cuda() const { return using_cuda_; }
  double last_detect_ms() const { return last_detect_ms_; }
  void reset_track();
  // Copy runtime scalars (thresholds, gates). Does not reload ONNX or cascades.
  void apply_runtime_cfg(const HumanDetectConfig &cfg);
  std::optional<PersonDetection> detect(const cv::Mat &bgr);

 private:
  std::optional<cv::Rect> detect_person_box(const cv::Mat &bgr);
  std::optional<PersonDetection> detect_face_in(const cv::Mat &bgr, const cv::Rect &roi);
  void warmup_person_net();
  static bool looks_like_person(const cv::Rect &r, int width, int height,
                                const HumanDetectConfig &cfg);

  struct PersonNet;
  HumanDetectConfig cfg_;
  cv::CascadeClassifier face_;
  std::unique_ptr<PersonNet> person_;
  cv::Rect last_person_;
  bool have_last_person_{false};
  bool ok_{false};
  bool person_ok_{false};
  bool using_cuda_{false};
  double last_detect_ms_{0.0};
};

// ---------------------------------------------------------------------------
// 3. Camera internals — turning a pixel into a 3D direction
// ---------------------------------------------------------------------------

// Pinhole camera. K is the 3×3 camera matrix:
//
//   [ fx  0  cx ]
//   [  0 fy  cy ]
//   [  0  0   1 ]
//
// fx, fy are focal lengths in pixels. (cx, cy) is the principal point —
// the pixel that looks straight along the lens axis.
//
// Do not hard-code 640×480 or fx = fy = 525 as "the" Aurora size. Live
// values come from /depth_cam/rgb/camera_info (width, height, k). The
// numbers below are the USB-camera yaml fallback
// (peripherals/config/camera_info.yaml), used only when camera_info has
// not arrived yet.
struct CameraIntrinsics {
  double fx{521.889};
  double fy{525.09003};
  double cx{339.68717};
  double cy{240.34345};
  int width{640};
  int height{480};
  bool from_camera_info{false};

  bool usable() const { return fx > 1.0 && fy > 1.0; }
};

CameraIntrinsics fallback_intrinsics();

// Build K from a row-major 3×3 (sensor_msgs/CameraInfo::k has 9 doubles).
// Returns an unusable struct if fx/fy are near zero — some drivers publish
// that before the first real calibration.
CameraIntrinsics from_k_matrix(int width, int height, const double k[9]);

// Pixel (u, v) → unit direction in the optical frame:
//
//   X = (u - cx) / fx
//   Y = (v - cy) / fy
//   ray = (X, Y, 1).normalized()
//
// A centre pixel (cx, cy) must give approximately (0, 0, 1) — straight
// out of the lens. Normalizing makes yaw and pitch independent of the
// arbitrary scale Z = 1. That Z is a virtual plane, not measured depth.
Eigen::Vector3d pixel_to_ray(double u, double v, const CameraIntrinsics &K);

// ---------------------------------------------------------------------------
// 4. Yaw and pitch — how far the person is from the image centre
// ---------------------------------------------------------------------------

// Angles that rotate the camera's +Z onto the target ray.
// Optical frame: X right, Y down, Z forward.
//
//   yaw   = atan2(X, Z)               // heading in the XZ plane
//   pitch = atan2(-Y, hypot(X, Z))    // the minus flips Y-down
//
// A pixel to the right of centre has X > 0, so yaw > 0.
// A pixel below centre has Y > 0, so pitch < 0.
// Units: radians. This is not arm inverse kinematics, and it is not the
// camera-from-base transform. It is only "how is this pixel off-centre?"
struct GazeAngles {
  double yaw{0.0};
  double pitch{0.0};
};

GazeAngles ray_to_yaw_pitch(const Eigen::Vector3d &ray);

// q = AngleAxis(yaw, UnitZ()) * AngleAxis(pitch, UnitY()).
// A centre pixel (yaw = pitch = 0) is the identity quaternion. The node
// logs this as a sanity check; it is never sent to a servo. The product
// must stay a unit quaternion (norm ≈ 1).
Eigen::Quaterniond gaze_quaternion(double yaw, double pitch);

// ---------------------------------------------------------------------------
// 5. Servo pulses — turning angles into bus-servo ticks
// ---------------------------------------------------------------------------

// Bus-servo travel on this robot: 240 degrees mapped onto pulse ticks 0..1000.
//
//   ticks_per_rad = 1000 / (240 * pi/180) = 750/pi ≈ 238.732
//
// Same constant as servo_controller/joint_position_controller.py
// (ENCODER_TICKS_PER_RADIAN). Ids 19 and 22 are not flipped in that driver.
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTicksPerRadian = 1000.0 / (240.0 * (kPi / 180.0));

// Rest pulses match proud_up / ActionGroups init_horizontal:
//   19 = 500  (joint 1, pan / yaw)
//   22 = 150  (wrist pitch; this joint is the camera's parent)
//
// There is no neck. Id 23 is joint 5, wrist *yaw*, not tilt. Never command
// 23 for pitch. Vendor color_track holds 23 and 24 at 500 and pans with 19;
// we do the same and put pitch on 22.
struct PulseMapping {
  float pan_rest{500.0f};    // servo 19 at zero yaw
  float tilt_rest{150.0f};   // servo 22 at zero pitch
  float pan_min{200.0f};
  float pan_max{800.0f};
  float tilt_min{80.0f};     // a tight band around 150 so the arm does not fold
  float tilt_max{250.0f};
  double yaw_sign{1.0};      // optical: +yaw = person to the right. Hardware yaml is -1
  double pitch_sign{1.0};
  double ticks_per_rad{kTicksPerRadian};
};

struct GazePulses {
  float id19{500.0f};
  float id22{150.0f};
};

// Absolute map, as if the camera were still at the rest pose:
//
//   pulse_19 = pan_rest  + yaw_sign   * yaw   * ticks_per_rad
//   pulse_22 = tilt_rest + pitch_sign * pitch * ticks_per_rad
//
// Clamped at the command site. This is the simple formula, and the unit
// tests use it. Do not command it every control cycle on a moving camera.
//
// Why: this formula pretends the camera is still at rest. If we sent it
// every tick, a person who reached the image centre would produce yaw = 0,
// so the arm would immediately return to rest. Then the person would be
// off-centre again, and the arm would chase them forever. That oscillation
// is why the node uses integrate_gaze instead.
GazePulses angles_to_pulses(const GazeAngles &angles, const PulseMapping &map);

// Incremental gaze — what the node actually publishes.
// yaw / pitch are the *current* optical error (how far the person is from
// the image centre *right now*). We add a fraction of that error onto the
// pulses we already have, and we hold still when the error is inside
// deadband_rad:
//
//   id19 += clamp(yaw_sign * yaw * ticks * gain, ±max_step)
//
// After the camera moves, the error shrinks and the pulses stay where they
// are. That is the same idea as color_track's `y_dis += pid.output`.
GazePulses integrate_gaze(const GazePulses &current, const GazeAngles &err,
                          const PulseMapping &map, double gain, double max_step,
                          double deadband_rad);

float clamp_pulse(float value, float lo, float hi);
float clamp_delta(float value, float max_abs);

// True when the person is close enough to the optical axis to count as
// "locked on centre" (the 2 s walk gate). The plan requires |yaw| small;
// pitch is included so a person standing a little high or low still
// qualifies. Default tolerances: yaw 0.08 rad ≈ 4.6°, pitch 0.20 rad ≈ 11.5°.
bool near_optical_axis(const GazeAngles &angles, double yaw_tol, double pitch_tol);

// ---------------------------------------------------------------------------
// 6. Coast, walk gate, debug overlay
// ---------------------------------------------------------------------------

// If YOLO misses for a moment, the node keeps the last box and marks it
// "coast" so the camera can hold still. Walking must not use that
// remembered box: the person may already have moved.
inline bool is_coasted_detection(const PersonDetection &det) {
  return det.label != nullptr && std::strcmp(det.label, "coast") == 0;
}

inline bool walk_detection_valid(const std::optional<PersonDetection> &det) {
  return det.has_value() && !is_coasted_detection(*det);
}

// Debug overlay for ~/image_result: box, aim pixel, phase string.
// Writes onto `bgr` in place (the node already owns a copy from toCvCopy).
void draw_debug_overlay(cv::Mat &bgr, const std::optional<PersonDetection> &det,
                        const char *phase, const GazeAngles *angles = nullptr,
                        const char *note = nullptr);

}  // namespace proud_up
