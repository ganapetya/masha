#pragma once

// Week 2 library: black portrait-rectangle detection + optical geometry + pulses.
//
// This header has NO rclcpp. The ROS node (follow_the_cat_node.cpp) calls these
// functions from a 10 Hz timer after it has copied the latest image pointer.
// Tests call the same functions with synthetic pixels / Mats — no Jetson needed.
//
// Optical frame (ROS camera convention, not the robot base):
//   X right, Y down, Z forward (out of the lens).
// A pixel (u, v) is not a 3D point; it is a ray. Depth is unknown this week.

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
// Day 1 — image plane
// ---------------------------------------------------------------------------

// Pixel coordinates in the Aurora RGB image. Origin is the top-left of the
// frame, u to the right, v down. Units: pixels (not metres).
struct Pixel {
  double u{0.0};
  double v{0.0};
};

// Target: a switched-off phone — a *black* rectangle taller than it is wide,
// held in front of Masha's white wall. Color alone is not enough (a dark
// doorway is also dark); we also require a 4-corner portrait box.
//
// `threshold` is a *darkness* cut: pixels darker than it become the mask
// (THRESH_BINARY_INV). Raise it if the phone looks grey under Aurora's dim
// exposure; lower it if furniture is being picked up. Week 5 is LAB color.
struct DetectionConfig {
  int threshold{90};          // gray < this counts as "black"
  int loose_threshold{130};   // second pass if the first is too strict
  double min_area{800.0};     // reject specks (px^2)
  double max_area{80000.0};   // hard cap; also limited by max_area_frac * pixels
  double max_area_frac{0.22}; // a wall filling the frame is not a phone
  double min_aspect{0.28};    // bounding-box width/height (phone ~0.45–0.55)
  double max_aspect{0.85};    // must stay portrait: height > width
  double min_solidity{0.55};  // contour area / bounding-box area
  int blur_ksize{5};          // Gaussian kernel; must be odd, or blur is skipped
  int morph_ksize{5};         // opening kernel; 0 disables
  int min_vertices{4};        // approxPolyDP window for a rectangle
  int max_vertices{6};        // phones have slightly rounded corners
  double poly_eps_frac{0.04}; // approxPolyDP epsilon as a fraction of perimeter
  int border_margin{4};       // px; wall-sized blobs touch the frame edge
  bool reject_border{true};   // drop contours that touch the image border
  bool require_quad{true};    // 4–6 vertices (a wall is not a small quad)
  bool require_portrait{true};  // bounding box must be taller than wide
};

// Result of detect_black_rectangle. `contour` / `approx` are in image
// coordinates so the node can draw them on ~/image_result.
struct CardDetection {
  Pixel center;
  std::vector<cv::Point> contour;
  std::vector<cv::Point> approx;
  double area{0.0};
  double aspect{0.0};  // width / height; < 1 means portrait
  const char *label{"target"};  // "face", "upper_body", "full_body", "phone"
};

// White walls are *bright*, so they never enter the inverted mask. A black
// portrait phone does. Empty / no-phone images return nullopt — a miss.
// Counts of rejected blobs (for ~/image_result) go in `stats` if non-null.
struct DetectionStats {
  int contours{0};
  int rejected_border{0};
  int rejected_area{0};
  int rejected_shape{0};
  bool used_loose_pass{false};
};

std::optional<CardDetection> detect_black_rectangle(const cv::Mat &bgr,
                                                    const DetectionConfig &cfg = {},
                                                    DetectionStats *stats = nullptr);

// Person gaze. YOLOv8n (COCO class 0) finds a person from any viewpoint —
// sitting, back to the camera, across the room. LBP face is a refinement
// inside that box when you actually look at the lens. Haar body cascades
// stay off: they were seconds-slow here and still missed a seated person.
struct HumanDetectConfig {
  std::string cascade_dir{"/usr/share/opencv4/haarcascades"};
  std::string lbp_dir{"/usr/share/opencv4/lbpcascades"};
  std::string person_onnx;  // empty = skip YOLO (tests / face-only)
  double image_scale{0.45};  // LBP face runs on a small gray crop
  double scale_factor{1.2};
  int min_neighbors{3};
  int min_face{24};   // px in the *original* image
  int min_upper{50};
  int min_full{60};
  bool enable_upper{false};
  bool enable_full{false};
  double person_conf{0.45};
  double person_iou{0.45};
  int person_imgsz{320};
  bool dnn_cuda{true};
  bool enable_face_refine{false};  // LBP inside the box jumps to curtains/windows
  double person_head_frac{0.45};   // torso, not the top of the box
  double max_box_frac{0.40};       // a curtain filling the frame is not a person
  double max_aspect{1.05};         // person is taller than wide
  double track_gate_frac{0.22};    // stay on the same person; do not hop
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
  // Copy runtime scalars (thresholds, gates). Does not reload ONNX / cascades.
  void apply_runtime_cfg(const HumanDetectConfig &cfg);
  std::optional<CardDetection> detect(const cv::Mat &bgr);

 private:
  std::optional<cv::Rect> detect_person_box(const cv::Mat &bgr);
  std::optional<CardDetection> detect_face_in(const cv::Mat &bgr, const cv::Rect &roi);
  void warmup_person_net();
  static bool looks_like_person(const cv::Rect &r, int width, int height,
                                const HumanDetectConfig &cfg);

  struct PersonNet;
  HumanDetectConfig cfg_;
  cv::CascadeClassifier face_;
  cv::CascadeClassifier upper_;
  cv::CascadeClassifier full_;
  std::unique_ptr<PersonNet> person_;
  cv::Rect last_person_;
  bool have_last_person_{false};
  bool ok_{false};
  bool person_ok_{false};
  bool using_cuda_{false};
  bool have_upper_{false};
  bool have_full_{false};
  double last_detect_ms_{0.0};
};

// ---------------------------------------------------------------------------
// Day 2 — camera intrinsics and the pixel → ray map
// ---------------------------------------------------------------------------

// Pinhole model. K is the 3×3 camera matrix:
//   [ fx  0  cx ]
//   [  0 fy  cy ]
//   [  0  0   1 ]
//
// Do NOT hardcode 640×480 / fx=fy=525 as "the" Aurora size. Live values come
// from /depth_cam/rgb/camera_info (msg.width, msg.height, msg.k). The numbers
// below are the USB-cam yaml fallback (peripherals/config/camera_info.yaml)
// used only when camera_info has not arrived yet.
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
// Returns a default-unusable struct if fx/fy are ~0 (some drivers publish that
// before the first real calibration).
CameraIntrinsics from_k_matrix(int width, int height, const double k[9]);

// Pixel (u, v) → unit direction in the optical frame:
//   X = (u - cx) / fx
//   Y = (v - cy) / fy
//   ray = (X, Y, 1).normalized()
// A center pixel (cx, cy) must give approximately (0, 0, 1) — the optical axis.
Eigen::Vector3d pixel_to_ray(double u, double v, const CameraIntrinsics &K);

// ---------------------------------------------------------------------------
// Day 3 — yaw / pitch in the optical frame, quaternion check, pulses
// ---------------------------------------------------------------------------

// Angles that rotate the camera's +Z onto the target ray.
// Optical: X right, Y down, Z forward.
//   yaw   = atan2(X, Z)              // heading in the XZ plane; right pixel → yaw > 0
//   pitch = atan2(-Y, hypot(X, Z))   // minus flips Y-down so below-center → pitch < 0
// Units: radians. This is NOT arm IK and NOT camera-from-base extrinsics.
struct GazeAngles {
  double yaw{0.0};
  double pitch{0.0};
};

GazeAngles ray_to_yaw_pitch(const Eigen::Vector3d &ray);

// q = AngleAxis(yaw, UnitZ()) * AngleAxis(pitch, UnitY()).
// A center pixel (yaw=pitch=0) is identity. Logged by the node; never sent
// to a servo. Norm must stay ~1 (unit quaternion).
Eigen::Quaterniond gaze_quaternion(double yaw, double pitch);

// Bus-servo travel on this robot: 240 deg mapped onto pulse ticks 0..1000.
//   ticks_per_rad = 1000 / (240 * pi/180) = 750/pi ≈ 238.732
// Same constant as servo_controller/joint_position_controller.py
// (ENCODER_TICKS_PER_RADIAN). Ids 19 and 22 are NOT flipped in that driver.
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTicksPerRadian = 1000.0 / (240.0 * (kPi / 180.0));

// Rest pulses match proud_up / ActionGroups init_horizontal:
//   19=500 (joint1 yaw), 22=150 (wrist pitch, camera parent).
// There is no neck. Id 23 is joint5 wrist *yaw*, not tilt — never command 23
// for pitch. Color_track holds 23 and 24 at 500 and pans with 19; we do the
// same and put pitch on 22.
struct PulseMapping {
  float pan_rest{500.0f};    // servo 19 at zero yaw
  float tilt_rest{150.0f};   // servo 22 at zero pitch
  float pan_min{200.0f};     // color_track clamp
  float pan_max{800.0f};
  float tilt_min{80.0f};     // tight band around 150 so the arm does not fold
  float tilt_max{250.0f};
  double yaw_sign{1.0};      // optical: +yaw = target to the right. Hardware: node yaml is -1
  double pitch_sign{1.0};
  double ticks_per_rad{kTicksPerRadian};
};

struct GazePulses {
  float id19{500.0f};
  float id22{150.0f};
};

// Absolute map, as if the camera were still at the rest pose:
//   pulse_19 = pan_rest  + yaw_sign   * yaw   * ticks_per_rad
//   pulse_22 = tilt_rest + pitch_sign * pitch * ticks_per_rad
// Clamped at the command site. This is the Week 2 formula and the gtest
// target. Do NOT command it every 10 Hz on a moving camera: when the card
// reaches the image center, yaw=0 so this snaps back to rest, then the card
// is off-center again — that is the hunting / "chaotic neck" loop.
GazePulses angles_to_pulses(const GazeAngles &angles, const PulseMapping &map);

// Incremental gaze (what the node actually publishes).
// yaw/pitch are the *current* optical error. We add a fraction of that error
// onto the *current* pulses and hold when the error is inside `deadband_rad`:
//   id19 += clamp(yaw_sign * yaw * ticks * gain, ±max_step)
// After the camera moves, the error shrinks and the pulses stay put. That is
// the same idea as color_track's `y_dis += pid.output`.
GazePulses integrate_gaze(const GazePulses &current, const GazeAngles &err,
                          const PulseMapping &map, double gain, double max_step,
                          double deadband_rad);

float clamp_pulse(float value, float lo, float hi);
float clamp_delta(float value, float max_abs);

// True when the blob is close enough to the optical axis to count as "locked
// on center" (the 2 s walk gate). Plan requires |yaw| small; pitch is included
// so a card held a bit high/low still qualifies.
bool near_optical_axis(const GazeAngles &angles, double yaw_tol, double pitch_tol);

// Debug overlay for ~/image_result: contour, centroid, phase string.
// Mutates `bgr` in place (the node already owns a copy from toCvCopy).
void draw_debug_overlay(cv::Mat &bgr, const std::optional<CardDetection> &det,
                        const char *phase, const GazeAngles *angles = nullptr,
                        const DetectionStats *stats = nullptr,
                        const char *note = nullptr);

// Coast fills a dropped YOLO box for gaze hold. Walk must not use that box.
inline bool is_coasted_detection(const CardDetection &det) {
  return det.label != nullptr && std::strcmp(det.label, "coast") == 0;
}

inline bool walk_detection_valid(const std::optional<CardDetection> &det) {
  return det.has_value() && !is_coasted_detection(*det);
}

}  // namespace proud_up
