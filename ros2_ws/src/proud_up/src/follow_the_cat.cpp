#include "proud_up/follow_the_cat.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <opencv2/imgproc.hpp>

namespace proud_up {

// ---------------------------------------------------------------------------
// Camera internals
// ---------------------------------------------------------------------------

CameraIntrinsics fallback_intrinsics() {
  // peripherals/config/camera_info.yaml — USB camera numbers, not a promise
  // about Aurora resolution_mode_index=2. Prefer live camera_info whenever
  // it exists.
  return CameraIntrinsics{};
}

CameraIntrinsics from_k_matrix(int width, int height, const double k[9]) {
  CameraIntrinsics K;
  K.width = width;
  K.height = height;
  // sensor_msgs/CameraInfo.k is row-major 3×3:
  //   k[0]=fx  k[2]=cx
  //   k[4]=fy  k[5]=cy
  K.fx = k[0];
  K.fy = k[4];
  K.cx = k[2];
  K.cy = k[5];
  K.from_camera_info = K.usable();
  return K;
}

Eigen::Vector3d pixel_to_ray(double u, double v, const CameraIntrinsics &K) {
  const double fx = (K.fx > 1e-6) ? K.fx : 1.0;
  const double fy = (K.fy > 1e-6) ? K.fy : 1.0;
  const double X = (u - K.cx) / fx;
  const double Y = (v - K.cy) / fy;
  // Z = 1 is a virtual plane in front of the lens, not a measured depth.
  // Normalizing makes this a unit direction so yaw and pitch do not
  // depend on that arbitrary scale.
  return Eigen::Vector3d(X, Y, 1.0).normalized();
}

// ---------------------------------------------------------------------------
// Yaw and pitch
// ---------------------------------------------------------------------------

GazeAngles ray_to_yaw_pitch(const Eigen::Vector3d &ray) {
  GazeAngles a;
  const double X = ray.x();
  const double Y = ray.y();
  const double Z = ray.z();
  a.yaw = std::atan2(X, Z);
  a.pitch = std::atan2(-Y, std::hypot(X, Z));
  return a;
}

Eigen::Quaterniond gaze_quaternion(double yaw, double pitch) {
  // Eigen multiplies on the left: (q_yaw * q_pitch) * v applies pitch
  // first, then yaw. That matches "tilt the lens, then pan". Hardware
  // gets the two scalar angles, not this quaternion; we only log it.
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  return q;
}

// ---------------------------------------------------------------------------
// Servo pulses
// ---------------------------------------------------------------------------

float clamp_pulse(float value, float lo, float hi) {
  return std::max(lo, std::min(hi, value));
}

float clamp_delta(float value, float max_abs) {
  const float cap = std::abs(max_abs);
  return std::max(-cap, std::min(cap, value));
}

GazePulses angles_to_pulses(const GazeAngles &angles, const PulseMapping &map) {
  GazePulses out;
  const double ticks = (map.ticks_per_rad > 1.0) ? map.ticks_per_rad : kTicksPerRadian;
  out.id19 = clamp_pulse(
      static_cast<float>(map.pan_rest + map.yaw_sign * angles.yaw * ticks),
      map.pan_min, map.pan_max);
  out.id22 = clamp_pulse(
      static_cast<float>(map.tilt_rest + map.pitch_sign * angles.pitch * ticks),
      map.tilt_min, map.tilt_max);
  return out;
}

GazePulses integrate_gaze(const GazePulses &current, const GazeAngles &err,
                          const PulseMapping &map, double gain, double max_step,
                          double deadband_rad) {
  GazePulses out = current;
  double yaw = err.yaw;
  double pitch = err.pitch;
  if (std::abs(yaw) < deadband_rad) {
    yaw = 0.0;
  }
  if (std::abs(pitch) < deadband_rad) {
    pitch = 0.0;
  }
  const double ticks = (map.ticks_per_rad > 1.0) ? map.ticks_per_rad : kTicksPerRadian;
  const double g = (gain > 0.0) ? gain : 0.0;
  const float dpan =
      clamp_delta(static_cast<float>(map.yaw_sign * yaw * ticks * g), static_cast<float>(max_step));
  const float dtilt = clamp_delta(static_cast<float>(map.pitch_sign * pitch * ticks * g),
                                  static_cast<float>(max_step));
  out.id19 = clamp_pulse(current.id19 + dpan, map.pan_min, map.pan_max);
  out.id22 = clamp_pulse(current.id22 + dtilt, map.tilt_min, map.tilt_max);
  return out;
}

HeadFocus decide_head_focus(bool latched, double yaw, double pitch, double deadband_rad,
                            double pixel_shift_px, double relock_px) {
  HeadFocus out;
  const double band = (deadband_rad > 0.0) ? deadband_rad : 0.0;
  const double relock = (relock_px > 0.0) ? relock_px : 0.0;
  const bool centered = std::abs(yaw) <= band && std::abs(pitch) <= band;
  if (!latched) {
    out.gaze = !centered;
    out.latched = centered;
    return out;
  }
  // Already looking at her. A sitting cat jitters a few tens of pixels;
  // that must not restart the pan. A real step across the picture does.
  if (pixel_shift_px <= relock) {
    out.gaze = false;
    out.latched = true;
    return out;
  }
  out.gaze = !centered;
  out.latched = centered;
  return out;
}

bool near_optical_axis(const GazeAngles &angles, double yaw_tol, double pitch_tol) {
  return std::abs(angles.yaw) <= yaw_tol && std::abs(angles.pitch) <= pitch_tol;
}

void draw_debug_overlay(cv::Mat &bgr, const std::optional<SubjectDetection> &det,
                        const char *phase, const GazeAngles *angles, const char *note) {
  if (bgr.empty()) {
    return;
  }
  const cv::Point frame_center(bgr.cols / 2, bgr.rows / 2);
  cv::drawMarker(bgr, frame_center, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 24, 2);

  if (det) {
    std::vector<std::vector<cv::Point>> one{det->contour};
    cv::drawContours(bgr, one, 0, cv::Scalar(0, 255, 0), 2);
    if (det->approx.size() >= 3) {
      cv::polylines(bgr, det->approx, true, cv::Scalar(0, 255, 255), 2);
    }
    const cv::Point c(static_cast<int>(std::lround(det->center.u)),
                      static_cast<int>(std::lround(det->center.v)));
    cv::circle(bgr, c, 6, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::drawMarker(bgr, c, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 18, 1);
    cv::line(bgr, c, frame_center, cv::Scalar(255, 0, 255), 1);
  }

  const cv::Scalar text_bg(0, 0, 0);
  const cv::Scalar text_fg(0, 255, 255);
  auto put = [&](const std::string &line, int y) {
    cv::putText(bgr, line, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, text_bg, 3);
    cv::putText(bgr, line, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, text_fg, 1);
  };

  put(std::string("phase: ") + (phase ? phase : "?"), 22);
  if (det) {
    put(std::string(det->label ? det->label : "person") +
            "  u=" + std::to_string(static_cast<int>(det->center.u)) +
            " v=" + std::to_string(static_cast<int>(det->center.v)),
        44);
  } else {
    put("no person (stand in front of the camera)", 44);
  }
  if (angles) {
    put("yaw=" + std::to_string(angles->yaw * 180.0 / kPi) + " deg  pitch=" +
            std::to_string(angles->pitch * 180.0 / kPi) + " deg",
        66);
  }
  if (note && note[0] != '\0') {
    put(note, 110);
  }
}

}  // namespace proud_up
