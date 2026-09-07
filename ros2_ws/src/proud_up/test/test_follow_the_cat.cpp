#include <cmath>
#include <optional>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "proud_up/follow_the_cat.hpp"

using proud_up::angles_to_pulses;
using proud_up::CameraIntrinsics;
using proud_up::clamp_pulse;
using proud_up::detect_black_rectangle;
using proud_up::DetectionConfig;
using proud_up::HumanDetector;
using proud_up::fallback_intrinsics;
using proud_up::from_k_matrix;
using proud_up::gaze_quaternion;
using proud_up::GazeAngles;
using proud_up::GazePulses;
using proud_up::integrate_gaze;
using proud_up::DetectionStats;
using proud_up::kPi;
using proud_up::kTicksPerRadian;
using proud_up::near_optical_axis;
using proud_up::pixel_to_ray;
using proud_up::PulseMapping;
using proud_up::ray_to_yaw_pitch;
using proud_up::walk_detection_valid;

namespace {

CameraIntrinsics unit_k() {
  CameraIntrinsics K;
  K.fx = 525.0;
  K.fy = 525.0;
  K.cx = 320.0;
  K.cy = 240.0;
  K.width = 640;
  K.height = 480;
  K.from_camera_info = true;
  return K;
}

cv::Mat white_wall() { return cv::Mat(400, 640, CV_8UC3, cv::Scalar(240, 240, 240)); }

}  // namespace

// Day 2: the principal point is the optical axis. After normalizing (X,Y,1)
// we must land on +Z of the optical frame, not some scaled (0,0,1) leftover.
TEST(FollowTheCat, CenterPixelRayIsOpticalAxis) {
  const auto K = unit_k();
  const Eigen::Vector3d ray = pixel_to_ray(K.cx, K.cy, K);
  EXPECT_NEAR(ray.x(), 0.0, 1e-9);
  EXPECT_NEAR(ray.y(), 0.0, 1e-9);
  EXPECT_NEAR(ray.z(), 1.0, 1e-9);
  EXPECT_NEAR(ray.norm(), 1.0, 1e-9);
}

// Day 2 / Day 3: a pixel to the right of cx has X>0, so atan2(X,Z) > 0.
// This is the optical-frame convention; hardware yaw_sign may still flip it.
TEST(FollowTheCat, RightPixelYawIsPositive) {
  const auto K = unit_k();
  const Eigen::Vector3d ray = pixel_to_ray(K.cx + 80.0, K.cy, K);
  EXPECT_GT(ray.x(), 0.0);
  const GazeAngles a = ray_to_yaw_pitch(ray);
  EXPECT_GT(a.yaw, 0.0);
  EXPECT_NEAR(a.pitch, 0.0, 1e-9);
}

// Optical Y is down, so a pixel below cy has Y>0 and pitch = atan2(-Y, ·) < 0.
TEST(FollowTheCat, BelowCenterPixelPitchIsNegative) {
  const auto K = unit_k();
  const Eigen::Vector3d ray = pixel_to_ray(K.cx, K.cy + 60.0, K);
  EXPECT_GT(ray.y(), 0.0);
  const GazeAngles a = ray_to_yaw_pitch(ray);
  EXPECT_LT(a.pitch, 0.0);
  EXPECT_NEAR(a.yaw, 0.0, 1e-9);
}

// Day 3: zero angles sit on the proud-up rest pulses (19=500, 22=150).
TEST(FollowTheCat, ZeroAnglesMapToRestPulses) {
  PulseMapping map;
  const GazePulses pulses = angles_to_pulses(GazeAngles{}, map);
  EXPECT_FLOAT_EQ(pulses.id19, 500.0f);
  EXPECT_FLOAT_EQ(pulses.id22, 150.0f);
}

// Safety: a huge yaw/pitch must not escape the joint clamps at the command site.
TEST(FollowTheCat, PulsesClampToJointLimits) {
  PulseMapping map;
  GazeAngles huge;
  huge.yaw = 2.0;     // ~115 deg, well past the 200–800 pan window
  huge.pitch = -1.5;  // would drive tilt far below 80
  const GazePulses pulses = angles_to_pulses(huge, map);
  EXPECT_GE(pulses.id19, map.pan_min);
  EXPECT_LE(pulses.id19, map.pan_max);
  EXPECT_GE(pulses.id22, map.tilt_min);
  EXPECT_LE(pulses.id22, map.tilt_max);
  EXPECT_FLOAT_EQ(pulses.id19, map.pan_max);  // +yaw, +sign → hits pan_max
  EXPECT_FLOAT_EQ(pulses.id22, map.tilt_min);

  EXPECT_FLOAT_EQ(clamp_pulse(10.0f, 80.0f, 250.0f), 80.0f);
  EXPECT_FLOAT_EQ(clamp_pulse(900.0f, 200.0f, 800.0f), 800.0f);
}

// Day 3 quaternion check: center pixel → identity, and the product stays unit.
TEST(FollowTheCat, CenterPixelQuaternionIsIdentity) {
  const auto K = unit_k();
  const GazeAngles a = ray_to_yaw_pitch(pixel_to_ray(K.cx, K.cy, K));
  const Eigen::Quaterniond q = gaze_quaternion(a.yaw, a.pitch);
  EXPECT_NEAR(q.norm(), 1.0, 1e-9);
  EXPECT_NEAR(q.w(), 1.0, 1e-9);
  EXPECT_NEAR(q.x(), 0.0, 1e-9);
  EXPECT_NEAR(q.y(), 0.0, 1e-9);
  EXPECT_NEAR(q.z(), 0.0, 1e-9);
  EXPECT_TRUE(near_optical_axis(a, 1e-6, 1e-6));
}

TEST(FollowTheCat, TicksPerRadianMatchesBusServoLaw) {
  // 1000 ticks over 240 deg = 750/pi.
  EXPECT_NEAR(kTicksPerRadian, 750.0 / kPi, 1e-9);
  EXPECT_NEAR(kTicksPerRadian, 238.732, 0.01);
}

TEST(FollowTheCat, FromKMatrixReadsCameraInfoLayout) {
  const double k[9] = {521.889, 0.0, 339.687, 0.0, 525.090, 240.343, 0.0, 0.0, 1.0};
  const CameraIntrinsics K = from_k_matrix(640, 400, k);
  EXPECT_TRUE(K.usable());
  EXPECT_TRUE(K.from_camera_info);
  EXPECT_EQ(K.width, 640);
  EXPECT_EQ(K.height, 400);
  EXPECT_DOUBLE_EQ(K.fx, 521.889);
  EXPECT_DOUBLE_EQ(K.fy, 525.090);
  EXPECT_DOUBLE_EQ(K.cx, 339.687);
  EXPECT_DOUBLE_EQ(K.cy, 240.343);
}

TEST(FollowTheCat, FallbackIntrinsicsMatchUsbCamYaml) {
  const CameraIntrinsics K = fallback_intrinsics();
  EXPECT_NEAR(K.fx, 521.889, 1e-3);
  EXPECT_NEAR(K.fy, 525.090, 1e-3);
  EXPECT_EQ(K.width, 640);
  EXPECT_EQ(K.height, 480);
  EXPECT_FALSE(K.from_camera_info);
}

// Switched-off phone: black, taller than wide, on a white wall.
TEST(FollowTheCat, DetectsBlackPortraitPhoneOnWhiteWall) {
  cv::Mat img = white_wall();
  const cv::Rect phone(280, 80, 80, 200);  // 80×200, center (320, 180)
  cv::rectangle(img, phone, cv::Scalar(20, 20, 20), cv::FILLED);

  const auto det = detect_black_rectangle(img);
  ASSERT_TRUE(det.has_value());
  EXPECT_NEAR(det->center.u, 320.0, 8.0);
  EXPECT_NEAR(det->center.v, 180.0, 8.0);
  EXPECT_LT(det->aspect, 1.0);
  EXPECT_GT(det->area, 10000.0);
}

TEST(FollowTheCat, LandscapeBlackRectangleIsRejected) {
  cv::Mat img = white_wall();
  cv::rectangle(img, cv::Rect(180, 160, 280, 80), cv::Scalar(20, 20, 20), cv::FILLED);
  EXPECT_FALSE(detect_black_rectangle(img).has_value());
}

TEST(FollowTheCat, WhiteRectangleOnWhiteWallIsIgnored) {
  cv::Mat img = white_wall();
  cv::rectangle(img, cv::Rect(280, 80, 80, 200), cv::Scalar(250, 250, 250), cv::FILLED);
  EXPECT_FALSE(detect_black_rectangle(img).has_value());
}

TEST(FollowTheCat, EmptyWhiteWallHasNoTarget) {
  DetectionStats stats;
  EXPECT_FALSE(detect_black_rectangle(white_wall(), {}, &stats).has_value());
}

TEST(FollowTheCat, FullBlackFrameIsRejectedAsNotAPhone) {
  const cv::Mat img(400, 640, CV_8UC3, cv::Scalar(10, 10, 10));
  EXPECT_FALSE(detect_black_rectangle(img).has_value());
}

TEST(FollowTheCat, EmptyMatYieldsNoDetection) {
  EXPECT_FALSE(detect_black_rectangle(cv::Mat()).has_value());
}

TEST(FollowTheCat, HumanDetectorLoadsCascades) {
  HumanDetector human;
  ASSERT_TRUE(human.ok());
  EXPECT_FALSE(human.person_ok());
  EXPECT_FALSE(human.detect(cv::Mat()).has_value());
  EXPECT_FALSE(human.detect(white_wall()).has_value());
}

TEST(FollowTheCat, YoloPersonIgnoresWhiteWall) {
  proud_up::HumanDetectConfig cfg;
  cfg.dnn_cuda = false;
  const char *candidates[] = {
      "/home/ubuntu/ros2_ws/src/proud_up/models/yolov8n.onnx",
      "/home/ubuntu/ros2_ws/install/proud_up/share/proud_up/models/yolov8n.onnx",
  };
  for (const char *path : candidates) {
    cfg.person_onnx = path;
    HumanDetector human(cfg);
    if (!human.person_ok()) {
      continue;
    }
    EXPECT_FALSE(human.detect(white_wall()).has_value());
    EXPECT_FALSE(human.detect(cv::Mat()).has_value());
    return;
  }
  GTEST_SKIP() << "yolov8n.onnx not installed";
}

TEST(FollowTheCat, IntegrateGazeHoldsOnZeroError) {
  PulseMapping map;
  GazePulses current;
  current.id19 = 560.0f;
  current.id22 = 170.0f;
  const GazePulses next = integrate_gaze(current, GazeAngles{}, map, 0.3, 18.0, 0.03);
  EXPECT_FLOAT_EQ(next.id19, 560.0f);
  EXPECT_FLOAT_EQ(next.id22, 170.0f);
}

TEST(FollowTheCat, IntegrateGazeStepsFromCurrentNotRest) {
  PulseMapping map;
  GazePulses current;
  current.id19 = 500.0f;
  GazeAngles err;
  err.yaw = 0.2;  // ~11 deg, well above deadband
  const GazePulses next = integrate_gaze(current, err, map, 0.3, 18.0, 0.03);
  EXPECT_GT(next.id19, current.id19);
  EXPECT_NE(next.id19, 500.0f + static_cast<float>(0.2 * kTicksPerRadian));
  EXPECT_LE(next.id19 - current.id19, 18.0f + 1e-3f);
}

TEST(FollowTheCat, IntegrateGazeSlewLimit) {
  PulseMapping map;
  GazePulses current;
  current.id19 = 500.0f;
  GazeAngles err;
  err.yaw = 1.0;  // huge optical error
  const GazePulses next = integrate_gaze(current, err, map, 1.0, 18.0, 0.0);
  EXPECT_NEAR(next.id19, 518.0f, 0.01);
}

TEST(FollowTheCat, CoastDetectionIsNotValidForWalk) {
  EXPECT_FALSE(walk_detection_valid(std::nullopt));

  proud_up::CardDetection person;
  person.label = "person";
  EXPECT_TRUE(walk_detection_valid(person));

  proud_up::CardDetection face;
  face.label = "face";
  EXPECT_TRUE(walk_detection_valid(face));

  proud_up::CardDetection coast;
  coast.label = "coast";
  EXPECT_FALSE(walk_detection_valid(coast));
}

TEST(FollowTheCat, YawSignFlipsPanPulseDirection) {
  PulseMapping map;
  GazeAngles a;
  a.yaw = 0.1;  // ~5.7 deg right in the optical frame
  const auto plus = angles_to_pulses(a, map);
  map.yaw_sign = -1.0;
  const auto minus = angles_to_pulses(a, map);
  EXPECT_GT(plus.id19, map.pan_rest);
  EXPECT_LT(minus.id19, map.pan_rest);
}
