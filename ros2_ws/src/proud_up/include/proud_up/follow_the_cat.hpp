#pragma once

// Follow-the-cat geometry: turn an aim pixel into pan/tilt servo pulses.
//
// The box itself comes from SubjectDetector (subject_detector.hpp). This
// header has no ROS and does not run YOLO. The node
// (follow_the_cat_node.cpp) calls these functions from a timer after it
// has a SubjectDetection. Tests call the same functions with fake pixels.
//
// Pipeline a learner can follow top to bottom:
//
//   aim pixel  →  ray through the camera lens
//             →  yaw / pitch (how far the subject is from the image centre)
//             →  a small step on arm servos 19 (pan) and 22 (tilt)
//
// Optical frame (ROS camera convention, not the robot base):
//   X right, Y down, Z forward (out of the lens).
// A pixel (u, v) is not a 3D point in the room. It is a direction.

#include <cmath>
#include <cstring>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "proud_up/subject_detector.hpp"

namespace proud_up {

// ---------------------------------------------------------------------------
// 1. Camera internals — turning a pixel into a 3D direction
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
// 2. Yaw and pitch — how far the subject is from the image centre
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
// 3. Servo pulses — turning angles into bus-servo ticks
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

// Head servo once a cat is already in the picture. The hunt triangle must
// not keep sweeping across a cat that is sitting still.
//
//   not latched, off centre  → gaze (one integrate_gaze step) until focused
//   not latched, inside deadband → latch and hold
//   latched, pixel shift ≤ relock_px → hold, even if the box jitters
//   latched, pixel shift > relock_px → the cat moved; aim again, then latch
//
// pixel_shift_px is the distance from the pixel where we latched. Pass 0
// when there is no latch yet. A latched head ignores optical error on
// purpose: a still cat must not make servo 19 hunt.
struct HeadFocus {
  bool gaze{false};     // true: take one integrate_gaze step this tick
  bool latched{false};  // true: republish the same pulses; do not scan
};

HeadFocus decide_head_focus(bool latched, double yaw, double pitch, double deadband_rad,
                            double pixel_shift_px, double relock_px);

float clamp_pulse(float value, float lo, float hi);
float clamp_delta(float value, float max_abs);

// True when the person is close enough to the optical axis to count as
// "locked on centre" (the 2 s walk gate). The plan requires |yaw| small;
// pitch is included so a person standing a little high or low still
// qualifies. Default tolerances: yaw 0.08 rad ≈ 4.6°, pitch 0.20 rad ≈ 11.5°.
bool near_optical_axis(const GazeAngles &angles, double yaw_tol, double pitch_tol);

// ---------------------------------------------------------------------------
// 4. Coast, walk gate, debug overlay
// ---------------------------------------------------------------------------

// If YOLO misses for a moment, the node keeps the last box and marks it
// "coast" so the camera can hold still. Walking must not use that
// remembered box: the person may already have moved.
inline bool is_coasted_detection(const SubjectDetection &det) {
  return det.label != nullptr && std::strcmp(det.label, "coast") == 0;
}

inline bool walk_detection_valid(const std::optional<SubjectDetection> &det) {
  return det.has_value() && !is_coasted_detection(*det);
}

// Debug overlay for ~/image_result: box, aim pixel, phase string.
// Writes onto `bgr` in place (the node already owns a copy from toCvCopy).
void draw_debug_overlay(cv::Mat &bgr, const std::optional<SubjectDetection> &det,
                        const char *phase, const GazeAngles *angles = nullptr,
                        const char *note = nullptr);

}  // namespace proud_up
