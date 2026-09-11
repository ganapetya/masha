// follow_the_cat_node — find a person and look at them, on Masha
// (Jetson Orin NX, ROS 2 Humble).
//
// Pipeline:
//
//   image → YOLO person box → aim pixel (torso)
//        → ray → yaw / pitch
//        → small servo step on ids 19 (pan) and 22 (tilt)
//        → optional 3 s walk at 0.05 m/s after 2 s of centre lock
//
// Math lives in follow_the_cat.hpp (no ROS). This file is the wiring:
// image subscription, 20 Hz detect and control timers, mutex, services,
// servo and cmd_vel publishers.
//
// Threads. Four mutually exclusive callback groups on a
// MultiThreadedExecutor, so they can overlap:
//   1. image / camera_info callbacks only store shared_ptrs under mutex_.
//      They do not run OpenCV. A slow detector on this thread would stall DDS.
//   2. detect_tick runs YOLO / LBP and writes the latest box.
//   3. control_tick runs the phase machine, servos, and walk.
//   4. watchdog_tick stops the legs if cmd_vel goes quiet while walking.
//
// Gaze is incremental (integrate_gaze), not "rest pose plus the current
// angle". Sending rest+angle every tick makes the arm oscillate: a person
// at the image centre produces yaw = 0, the arm returns to rest, then the
// person is off-centre again.
//
// Safety. enable_walk defaults to false so first bring-up is gaze only.
// Do not publish a Twist with linear.x = 0 to /controller/cmd_vel — the
// gait generator still starts and the robot steps in place. To stop the
// legs, publish Traveling gait 0 (stop) then gait -2 (stand). Only arm
// ids 19–24 are commanded.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <kinematics_msgs/msg/traveling.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "proud_up/follow_the_cat.hpp"
#include "proud_up/pose_commander.hpp"

using namespace std::chrono_literals;

namespace proud_up {
namespace {

// Phase machine (one control tick at a time):
//
//   IDLE ----confirm_frames----> TRACKING --2 s centred + enable_walk--> MOVING
//     ^                            | miss for lost_timeout                  |
//     |                            v                                        |
//     +-------------------------- LOST <----- miss / coast / stale ---------+
//                                                                           |
//                                                    walk_seconds elapsed   v
//                                                                         DONE
//
// IDLE:      rest pose, wait for a person. Never publish cmd_vel.
// TRACKING:  pan / tilt onto the person. Walk is only *allowed* here, not started
//            until the person has stayed near the image centre for lock_seconds.
// MOVING:    publish linear.x = walk_speed for at most walk_seconds. Still aim.
// DONE:      hold gaze. Legs are already stopped. Stays here until ~/start.
// LOST:      person gone. Next tick returns to IDLE. Aborts a walk if one was
//            running. Keeps the last pan / tilt (does not return to rest).
enum class Phase { Idle, Tracking, Moving, Done, Lost };

const char *phase_name(Phase phase) {
  switch (phase) {
    case Phase::Idle:
      return "idle";
    case Phase::Tracking:
      return "tracking";
    case Phase::Moving:
      return "moving";
    case Phase::Done:
      return "done";
    case Phase::Lost:
      return "lost";
  }
  return "unknown";
}

// Latest person as seen by the control timer, plus the ROS stamp of that
// image. Guarded by the node mutex together with the image / camera_info
// pointers. The image callback does not write this — it only stores the
// Image pointer (a cheap shared_ptr copy). Detection runs on the timer
// thread after the lock is released, then the result is written back.
struct TargetState {
  double u{0.0};
  double v{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool seen{false};
};

}  // namespace

class FollowTheCatNode : public rclcpp::Node {
 public:
  FollowTheCatNode() : Node("follow_the_cat_node") {
    image_topic_ = declare_parameter<std::string>("image_topic", "/depth_cam/rgb/image_raw");
    camera_info_topic_ =
        declare_parameter<std::string>("camera_info_topic", "/depth_cam/rgb/camera_info");
    servo_topic_ = declare_parameter<std::string>("servo_topic", "servo_controller");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/controller/cmd_vel");
    controller_ready_service_ =
        declare_parameter<std::string>("controller_ready_service", "/controller_manager/init_finish");
    init_pose_ready_service_ =
        declare_parameter<std::string>("init_pose_ready_service", "/init_pose/init_finish");

    wait_for_ready_ = declare_parameter<bool>("wait_for_ready", true);
    image_qos_reliable_ = declare_parameter<bool>("image_qos_reliable", false);
    enable_walk_ = declare_parameter<bool>("enable_walk", false);
    dry_run_ = declare_parameter<bool>("dry_run", false);

    human_cfg_.cascade_dir = declare_parameter<std::string>(
        "cascade_dir", "/usr/share/opencv4/haarcascades");
    human_cfg_.lbp_dir = declare_parameter<std::string>(
        "lbp_dir", "/usr/share/opencv4/lbpcascades");
    human_cfg_.image_scale = declare_parameter<double>("human_image_scale", 0.45);
    human_cfg_.scale_factor = declare_parameter<double>("human_scale_factor", 1.2);
    human_cfg_.min_neighbors = declare_parameter<int>("human_min_neighbors", 3);
    human_cfg_.min_face = declare_parameter<int>("min_face", 24);
    human_cfg_.person_conf = declare_parameter<double>("person_conf", 0.45);
    human_cfg_.person_iou = declare_parameter<double>("person_iou", 0.45);
    human_cfg_.person_imgsz = declare_parameter<int>("person_imgsz", 320);
    human_cfg_.dnn_cuda = declare_parameter<bool>("dnn_cuda", true);
    human_cfg_.enable_face_refine = declare_parameter<bool>("enable_face_refine", false);
    human_cfg_.person_head_frac = declare_parameter<double>("person_head_frac", 0.45);
    human_cfg_.max_box_frac = declare_parameter<double>("max_box_frac", 0.40);
    human_cfg_.max_aspect = declare_parameter<double>("person_max_aspect", 1.05);
    human_cfg_.track_gate_frac = declare_parameter<double>("track_gate_frac", 0.22);
    human_cfg_.person_onnx = declare_parameter<std::string>("person_onnx", "");
    if (human_cfg_.person_onnx.empty()) {
      try {
        human_cfg_.person_onnx =
            ament_index_cpp::get_package_share_directory("proud_up") + "/models/yolov8n.onnx";
      } catch (const std::exception &) {
        human_cfg_.person_onnx.clear();
      }
    }
    if (!human_cfg_.person_onnx.empty()) {
      std::ifstream probe(human_cfg_.person_onnx);
      if (!probe.good()) {
        RCLCPP_ERROR(get_logger(), "person ONNX missing: %s", human_cfg_.person_onnx.c_str());
        human_cfg_.person_onnx.clear();
      }
    }
    human_ = std::make_unique<HumanDetector>(human_cfg_);
    if (!human_->ok()) {
      RCLCPP_ERROR(get_logger(),
                   "no person detector (cascade %s / %s, onnx %s) — gaze will miss",
                   human_cfg_.lbp_dir.c_str(), human_cfg_.cascade_dir.c_str(),
                   human_cfg_.person_onnx.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "person detector onnx=%s cuda=%s face=%s",
                  human_->person_ok() ? human_cfg_.person_onnx.c_str() : "off",
                  human_->using_cuda() ? "yes" : "no",
                  human_cfg_.cascade_dir.empty() ? "off" : "lbp");
    }

    pulse_map_.yaw_sign = declare_parameter<double>("yaw_sign", -1.0);
    pulse_map_.pitch_sign = declare_parameter<double>("pitch_sign", 1.0);
    pulse_map_.pan_min = static_cast<float>(declare_parameter<double>("pan_min", 200.0));
    pulse_map_.pan_max = static_cast<float>(declare_parameter<double>("pan_max", 800.0));
    pulse_map_.tilt_min = static_cast<float>(declare_parameter<double>("tilt_min", 80.0));
    pulse_map_.tilt_max = static_cast<float>(declare_parameter<double>("tilt_max", 250.0));
    pulse_map_.ticks_per_rad = kTicksPerRadian;

    rest_.id19 = static_cast<float>(declare_parameter<double>("arm_id19", 500.0));
    rest_.id20 = static_cast<float>(declare_parameter<double>("arm_id20", 810.0));
    rest_.id21 = static_cast<float>(declare_parameter<double>("arm_id21", 180.0));
    rest_.id22 = static_cast<float>(declare_parameter<double>("arm_id22", 150.0));
    rest_.id23 = static_cast<float>(declare_parameter<double>("arm_id23", 500.0));
    rest_.id24 = static_cast<float>(declare_parameter<double>("arm_id24", 500.0));
    rest_.duration_s = declare_parameter<double>("arm_duration", 1.0);
    pulse_map_.pan_rest = rest_.id19;
    pulse_map_.tilt_rest = rest_.id22;

    lock_seconds_ = declare_parameter<double>("lock_seconds", 2.0);
    lost_timeout_seconds_ = declare_parameter<double>("lost_timeout_seconds", 6.0);
    walk_seconds_ = declare_parameter<double>("walk_seconds", 3.0);
    walk_speed_ = declare_parameter<double>("walk_speed", 0.05);
    center_yaw_tol_ = declare_parameter<double>("center_yaw_tol", 0.08);
    center_pitch_tol_ = declare_parameter<double>("center_pitch_tol", 0.20);
    servo_duration_ = declare_parameter<double>("servo_duration", 0.08);
    ready_timeout_seconds_ = declare_parameter<double>("ready_timeout_seconds", 30.0);
    gaze_gain_ = declare_parameter<double>("gaze_gain", 0.18);
    gaze_ki_ = declare_parameter<double>("gaze_ki", 0.0);
    max_step_pulses_ = declare_parameter<double>("max_step_pulses", 8.0);
    deadband_rad_ = declare_parameter<double>("deadband_rad", 0.06);
    aim_u_offset_px_ = declare_parameter<double>("aim_u_offset_px", 0.0);
    aim_v_offset_px_ = declare_parameter<double>("aim_v_offset_px", 0.0);
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.30);
    confirm_frames_ = declare_parameter<int>("confirm_frames", 3);
    max_jump_px_ = declare_parameter<double>("max_jump_px", 180.0);
    debug_max_width_ = declare_parameter<int>("debug_max_width", 640);
    walk_watchdog_seconds_ = declare_parameter<double>("walk_watchdog_seconds", 0.3);

    gaze_id19_ = rest_.id19;
    gaze_id22_ = rest_.id22;

    // Mutually exclusive groups so image store, YOLO, control, and the walk
    // watchdog can overlap on a MultiThreadedExecutor. The default group is
    // mutually exclusive for the *whole node*, which would serialise them.
    image_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    detect_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    servo_pub_ = create_publisher<servo_controller_msgs::msg::ServosPosition>(servo_topic_, 1);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
    traveling_pub_ = create_publisher<kinematics_msgs::msg::Traveling>("/controller/traveling", 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_result", 1);

    // Aurora publishes best-effort. A default *reliable* subscription on
    // Humble often sees zero frames. SensorDataQoS is KeepLast(5)+best_effort.
    // camera_info is typically reliable; we still match the image QoS so a
    // best-effort info topic would work too.
    rclcpp::QoS image_qos = rclcpp::SensorDataQoS();
    if (image_qos_reliable_) {
      image_qos.reliable();
    }
    rclcpp::SubscriptionOptions image_opts;
    image_opts.callback_group = image_cb_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, image_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(std::move(msg)); },
        image_opts);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, image_qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { on_camera_info(std::move(msg)); },
        image_opts);

    controller_client_ = create_client<std_srvs::srv::Trigger>(controller_ready_service_);
    init_pose_client_ = create_client<std_srvs::srv::Trigger>(init_pose_ready_service_);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/start",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          on_start(response);
        });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          on_stop(response);
        });
    init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/init_finish",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          response->success = true;
          response->message = phase_name(phase_);
        });

    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          return on_set_parameters(params);
        });

    detect_timer_ = create_wall_timer(50ms, [this]() { detect_tick(); }, detect_cb_group_);
    control_timer_ = create_wall_timer(50ms, [this]() { control_tick(); }, control_cb_group_);
    watchdog_timer_ = create_wall_timer(100ms, [this]() { watchdog_tick(); }, watchdog_cb_group_);
    started_at_ = now();
    phase_started_at_ = started_at_;

    RCLCPP_INFO(get_logger(),
                "follow_the_cat image=%s info=%s servo=%s cmd_vel=%s "
                "enable_walk=%s dry_run=%s yaw_sign=%.1f pitch_sign=%.1f "
                "(tilt is servo 22, not 23)",
                image_topic_.c_str(), camera_info_topic_.c_str(), servo_topic_.c_str(),
                cmd_vel_topic_.c_str(), enable_walk_ ? "true" : "false",
                dry_run_ ? "true" : "false", pulse_map_.yaw_sign, pulse_map_.pitch_sign);
    if (!enable_walk_) {
      RCLCPP_INFO(get_logger(),
                  "walk is OFF (enable_walk:=false). First bring-up is gaze-only. "
                  "Confirm pan/tilt signs on hardware before enabling walk.");
    }
  }

  ~FollowTheCatNode() override { emergency_stop(); }

  // Called from main after spin, and from the destructor / SIGINT path.
  // Stops the legs with Traveling (gait 0 then -2), not a zero Twist.
  void emergency_stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (phase_ != Phase::Done) {
        phase_ = Phase::Done;
      }
      stopping_ = true;
    }
    halt_legs();
  }

 private:
  rclcpp::Time now() { return get_clock()->now(); }

  // Store the shared_ptr (refcount++, no pixel copy) and return. Do not
  // call OpenCV here — a slow detector would stall DDS.
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_image_ = std::move(msg);
  }

  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_info_ = std::move(msg);
  }

  // Vision half of the loop. Copy the latest image pointer, drop the lock,
  // run YOLO, then write the box and yaw/pitch back under the lock.
  // Control never waits on this; a slow detect only makes the box stale.
  void detect_tick() {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    bool stopping;
    HumanDetectConfig human_cfg;
    bool reset_track = false;
    double lost_timeout;
    double aim_u;
    double aim_v;
    Phase overlay_phase = Phase::Idle;
    {
      // Copy pointers and small values, then drop the lock BEFORE OpenCV.
      std::lock_guard<std::mutex> lock(mutex_);
      image = latest_image_;
      info = latest_info_;
      stopping = stopping_;
      human_cfg = human_cfg_;
      reset_track = reset_track_pending_;
      reset_track_pending_ = false;
      lost_timeout = lost_timeout_seconds_;
      aim_u = aim_u_offset_px_;
      aim_v = aim_v_offset_px_;
      overlay_phase = phase_;
    }

    if (stopping) {
      return;
    }

    if (reset_track && human_) {
      human_->reset_track();
    }
    if (human_) {
      human_->apply_runtime_cfg(human_cfg);
    }

    CameraIntrinsics K = fallback_intrinsics();
    if (info && info->k.size() >= 9 && info->width > 0 && info->height > 0) {
      const auto live = from_k_matrix(static_cast<int>(info->width),
                                      static_cast<int>(info->height), info->k.data());
      if (live.usable()) {
        K = live;
        if (!logged_intrinsics_) {
          logged_intrinsics_ = true;
          RCLCPP_INFO(get_logger(),
                      "camera_info %dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
                      "(live K; not the 640x480/fx=525 syllabus defaults)",
                      K.width, K.height, K.fx, K.fy, K.cx, K.cy);
        }
      }
    } else if (!logged_fallback_intrinsics_) {
      logged_fallback_intrinsics_ = true;
      RCLCPP_WARN(get_logger(),
                  "no usable /depth_cam/rgb/camera_info yet; using USB-cam yaml "
                  "fallback %dx%d fx=%.1f fy=%.1f — check `ros2 topic echo "
                  "/depth_cam/rgb/camera_info --once`",
                  K.width, K.height, K.fx, K.fy);
    }

    std::optional<PersonDetection> det;
    GazeAngles angles;
    cv::Mat annotated;
    rclcpp::Time image_stamp(0, 0, RCL_ROS_TIME);

    if (image) {
      image_stamp = rclcpp::Time(image->header.stamp);
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
      } catch (const cv_bridge::Exception &ex) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", ex.what());
        return;
      }
      if (cv_ptr && !cv_ptr->image.empty()) {
        annotated = cv_ptr->image;
        det = human_->detect(annotated);
        const double ms = human_->last_detect_ms();
        if (ms > 80.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "person detect took %.0f ms — control loop is free", ms);
        }
        {
          // If YOLO misses for a moment, keep the last box and mark it
          // "coast" so the camera can hold still. Walking must not use
          // that remembered box: the person may already have moved.
          std::lock_guard<std::mutex> lock(mutex_);
          if (det) {
            coast_det_ = det;
            last_fresh_at_ = now();
          } else if (coast_det_ && last_fresh_at_.nanoseconds() != 0 &&
                     (now() - last_fresh_at_).seconds() <= lost_timeout) {
            det = coast_det_;
            det->label = "coast";
          } else {
            coast_det_.reset();
            ema_ready_ = false;
          }
        }
        if (det) {
          Pixel center = det->center;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (ema_ready_) {
              const double du = center.u - ema_u_;
              const double dv = center.v - ema_v_;
              if (std::hypot(du, dv) > max_jump_px_) {
                det.reset();
              }
            }
            if (det) {
              if (ema_ready_) {
                ema_u_ = ema_alpha_ * center.u + (1.0 - ema_alpha_) * ema_u_;
                ema_v_ = ema_alpha_ * center.v + (1.0 - ema_alpha_) * ema_v_;
                center.u = ema_u_;
                center.v = ema_v_;
              } else {
                ema_u_ = center.u;
                ema_v_ = center.v;
                ema_ready_ = true;
              }
            }
          }
          if (det) {
            det->center = center;
            // Aim at the image centre (the magenta cross), not the
            // calibration principal point. aim_u / aim_v are small
            // pixel offsets if the cross is not where you want it.
            CameraIntrinsics aim = K;
            aim.cx = 0.5 * static_cast<double>(annotated.cols) + aim_u;
            aim.cy = 0.5 * static_cast<double>(annotated.rows) + aim_v;
            const Eigen::Vector3d ray = pixel_to_ray(center.u, center.v, aim);
            angles = ray_to_yaw_pitch(ray);
            const Eigen::Quaterniond q = gaze_quaternion(angles.yaw, angles.pitch);
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ray=(%.3f, %.3f, %.3f) yaw=%.3f rad pitch=%.3f rad q.norm=%.6f", ray.x(),
                ray.y(), ray.z(), angles.yaw, angles.pitch, q.norm());
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_det_ = det;
      latest_angles_ = angles;
      target_.seen = static_cast<bool>(det);
      if (det) {
        target_.u = det->center.u;
        target_.v = det->center.v;
        target_.stamp = image_stamp;
        if (!is_coasted_detection(*det)) {
          last_seen_at_ = now();
        }
      }
      overlay_phase = phase_;
    }

    if (!annotated.empty()) {
      char err_note[128];
      if (det) {
        const double du = det->center.u - 0.5 * annotated.cols - aim_u;
        std::snprintf(err_note, sizeof(err_note),
                      "du=%+.0f px  yaw=%+.1f deg  det=%.0fms", du,
                      angles.yaw * 180.0 / kPi,
                      human_ ? human_->last_detect_ms() : 0.0);
      } else {
        std::snprintf(err_note, sizeof(err_note), "stand in front of Masha, face the camera");
      }
      draw_debug_overlay(annotated, det, phase_name(overlay_phase), det ? &angles : nullptr,
                         err_note);
      publish_debug_image(annotated, image);
    }
  }

  // Behaviour half of the loop. Copy the latest box, run one phase tick
  // under the lock, then publish servos / cmd_vel *after* the lock drops.
  void control_tick() {
    maybe_mark_ready();

    std::optional<PersonDetection> det;
    GazeAngles angles;
    PulseMapping pulse_map;
    bool enable_walk;
    bool dry_run;
    bool stopping;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping = stopping_;
      det = latest_det_;
      angles = latest_angles_;
      pulse_map = pulse_map_;
      enable_walk = enable_walk_;
      dry_run = dry_run_;
    }

    if (stopping) {
      return;
    }

    geometry_msgs::msg::Twist twist;  // only published when linear.x != 0
    bool publish_twist = false;
    bool publish_servos = false;
    bool halt_after_unlock = false;
    ArmPulses arm = rest_;
    double servo_dt = servo_duration_;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        return;
      }

      if (hardware_ready_) {
        switch (phase_) {
          case Phase::Idle:
            tick_idle_locked(det, angles, pulse_map, arm, servo_dt, publish_servos);
            break;
          case Phase::Tracking:
            tick_tracking_locked(det, angles, pulse_map, enable_walk, arm, servo_dt,
                                 publish_servos);
            break;
          case Phase::Moving:
            tick_moving_locked(det, angles, pulse_map, arm, servo_dt, publish_servos, twist,
                               publish_twist, halt_after_unlock);
            break;
          case Phase::Lost:
            // Keep the last pan and tilt. Moving the arm back to rest as
            // soon as YOLO misses one frame looks like random looking-around.
            lock_active_ = false;
            seen_streak_ = 0;
            pan_i_ = 0.0;
            if (walking_active_) {
              halt_after_unlock = true;
            }
            set_phase_locked(Phase::Idle);
            break;
          case Phase::Done:
            break;
        }
      }

      if (stopping_) {
        publish_servos = false;
        publish_twist = false;
        halt_after_unlock = true;
      }
    }

    if (publish_servos) {
      if (dry_run) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "dry_run pulses id19=%.0f id22=%.0f (not sent)", arm.id19,
                             arm.id22);
      } else {
        servo_pub_->publish(make_arm_command(arm));
      }
    }
    // Never publish a zero Twist. vx = 0 still starts CmdVelGenerator
    // forever (in-place stepping). Only a real forward command is allowed.
    if (publish_twist && twist.linear.x != 0.0) {
      if (dry_run) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "dry_run walk linear.x=%.3f (not sent)", twist.linear.x);
      } else {
        cmd_vel_pub_->publish(twist);
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_walk_cmd_at_ = now();
      }
    }
    if (halt_after_unlock) {
      halt_legs();
    }
  }

  void set_phase_locked(Phase next) {
    if (phase_ == next) {
      return;
    }
    RCLCPP_INFO(get_logger(), "follow_the_cat: %s -> %s", phase_name(phase_), phase_name(next));
    phase_ = next;
    phase_started_at_ = now();
    if (next == Phase::Tracking) {
      lock_active_ = false;
    }
    if (next == Phase::Moving) {
      walk_started_at_ = now();
      last_walk_cmd_at_ = walk_started_at_;
    }
  }

  // Rest pose, wait for a person, never walk.
  //
  // First tick: send the rest pose once and seed the current pan / tilt
  // from rest. After that, do nothing until we have seen a person for
  // confirm_frames consecutive ticks (default 3). A single false
  // detection would otherwise send a large, sudden arm motion.
  //
  // A miss zeros the streak; we stay in Idle. Next phase: Tracking.
  void tick_idle_locked(const std::optional<PersonDetection> &det, const GazeAngles &angles,
                        const PulseMapping &pulse_map, ArmPulses &arm, double &servo_dt,
                        bool &publish_servos) {
    if (!rest_sent_) {
      arm = rest_;
      servo_dt = rest_.duration_s;
      arm.duration_s = servo_dt;
      publish_servos = true;
      rest_sent_ = true;
      gaze_id19_ = rest_.id19;
      gaze_id22_ = rest_.id22;
    }
    if (!det) {
      seen_streak_ = 0;
      return;
    }
    ++seen_streak_;
    if (seen_streak_ >= confirm_frames_) {
      publish_servos = apply_gaze(arm, servo_dt, angles, pulse_map);
      set_phase_locked(Phase::Tracking);
    }
  }

  // Keep the person on the magenta cross (image centre).
  //
  // Miss: hold the last pan / tilt. After lost_timeout_seconds (default 6 s)
  // with no detection, go to Lost. A coasted box still counts as "seen" for
  // the phase, so a one-frame YOLO blink does not drop us out — but we do
  // not drive the servos from a remembered box.
  //
  // Hit: apply_gaze on a fresh box. If the person stays near the optical
  // axis for lock_seconds (default 2 s) *continuously* and enable_walk is
  // true, go to Moving. Leaving the centre restarts the 2 s timer.
  //
  // This function never publishes cmd_vel. enable_walk only decides whether
  // Tracking may *enter* Moving.
  void tick_tracking_locked(const std::optional<PersonDetection> &det, const GazeAngles &angles,
                            const PulseMapping &pulse_map, bool enable_walk, ArmPulses &arm,
                            double &servo_dt, bool &publish_servos) {
    if (!det) {
      seen_streak_ = 0;
      if (lost_for_too_long()) {
        set_phase_locked(Phase::Lost);
      }
      return;
    }
    seen_streak_ = confirm_frames_;
    const bool coasting = is_coasted_detection(*det);
    if (!coasting) {
      publish_servos = apply_gaze(arm, servo_dt, angles, pulse_map);
    }

    const bool centered = near_optical_axis(angles, center_yaw_tol_, center_pitch_tol_);
    if (centered) {
      if (!lock_active_) {
        lock_active_ = true;
        lock_started_at_ = now();
      }
      const double held = (now() - lock_started_at_).seconds();
      if (enable_walk && held >= lock_seconds_ && std::abs(angles.yaw) <= center_yaw_tol_) {
        // Walk is a one-shot ~15 cm step, not a chase. enable_walk
        // defaults to false so we can verify pan / tilt signs first.
        set_phase_locked(Phase::Moving);
      }
    } else {
      lock_active_ = false;
    }
  }

  // One short forward walk, still aiming.
  //
  // Abort immediately (Lost, halt legs) if the box is missing, coasted, or
  // older than walk_watchdog_seconds (default 0.3 s). Walking on a frozen
  // YOLO box is unsafe — the person may have stepped aside.
  //
  // Otherwise keep applying gaze and publish linear.x = walk_speed
  // (0.05 m/s). After walk_seconds (default 3 s) halt the legs and go to
  // Done. Done does not walk again until ~/start.
  void tick_moving_locked(const std::optional<PersonDetection> &det, const GazeAngles &angles,
                          const PulseMapping &pulse_map, ArmPulses &arm, double &servo_dt,
                          bool &publish_servos, geometry_msgs::msg::Twist &twist,
                          bool &publish_twist, bool &halt_after_unlock) {
    const bool stale_box =
        last_fresh_at_.nanoseconds() == 0 ||
        (now() - last_fresh_at_).seconds() > walk_watchdog_seconds_;
    if (!walk_detection_valid(det) || stale_box) {
      halt_after_unlock = true;
      set_phase_locked(Phase::Lost);
      return;
    }
    publish_servos = apply_gaze(arm, servo_dt, angles, pulse_map);

    if ((now() - walk_started_at_).seconds() >= walk_seconds_) {
      halt_after_unlock = true;
      set_phase_locked(Phase::Done);
      RCLCPP_INFO(get_logger(), "walk complete (%.1f s at %.3f m/s); holding", walk_seconds_,
                  walk_speed_);
      return;
    }
    // move_controller clamps linear.x to ±0.12 m/s; 0.05 is legal.
    twist.linear.x = walk_speed_;
    publish_twist = true;
    walking_active_ = true;
  }

  // Convert the current optical error into a small servo step.
  //
  // Returns true only if a pulse actually moved by at least 1.5 ticks.
  // Re-sending the same 80 ms servo packet at 20 Hz makes the arm twitch
  // even when the person is already inside the deadband.
  bool apply_gaze(ArmPulses &arm, double &servo_dt, const GazeAngles &angles,
                  const PulseMapping &pulse_map) {
    const double dt = 0.05;
    if (std::abs(angles.yaw) > deadband_rad_) {
      pan_i_ += angles.yaw * dt;
    } else {
      pan_i_ *= 0.85;
    }
    pan_i_ = std::max(-0.5, std::min(0.5, pan_i_));
    GazeAngles err = angles;
    err.yaw += gaze_ki_ * pan_i_;

    const GazePulses current{gaze_id19_, gaze_id22_};
    const GazePulses next = integrate_gaze(current, err, pulse_map, gaze_gain_,
                                           max_step_pulses_, deadband_rad_);
    const bool moved = std::abs(next.id19 - current.id19) >= 1.5f ||
                       std::abs(next.id22 - current.id22) >= 1.5f;
    gaze_id19_ = next.id19;
    gaze_id22_ = next.id22;
    arm = rest_;
    arm.id19 = next.id19;  // pan, joint 1. Clamped 200–800 inside integrate_gaze.
    arm.id22 = next.id22;  // tilt, wrist pitch / camera parent. NOT id 23.
    servo_dt = servo_duration_;
    arm.duration_s = servo_dt;
    return moved;
  }

  bool lost_for_too_long() {
    if (last_seen_at_.nanoseconds() == 0) {
      return true;
    }
    return (now() - last_seen_at_).seconds() > lost_timeout_seconds_;
  }

  // If cmd_vel goes quiet while walking (detect or control stalled), stop
  // the legs. Same Traveling halt as a normal abort — never a zero Twist.
  void watchdog_tick() {
    bool should_halt = false;
    double timeout = 0.3;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      timeout = walk_watchdog_seconds_;
      if (stopping_ || !walking_active_ || timeout <= 0.0) {
        return;
      }
      if (last_walk_cmd_at_.nanoseconds() == 0) {
        return;
      }
      if ((now() - last_walk_cmd_at_).seconds() <= timeout) {
        return;
      }
      walking_active_ = false;
      lock_active_ = false;
      set_phase_locked(Phase::Lost);
      should_halt = true;
    }
    if (should_halt) {
      halt_legs();
      RCLCPP_WARN(get_logger(),
                  "walk watchdog: no cmd_vel for %.2f s; halt_legs (Traveling, not zero Twist)",
                  timeout);
    }
  }

  void send_rest_pose(double duration_s) {
    ArmPulses pose = rest_;
    pose.duration_s = duration_s;
    if (dry_run_) {
      RCLCPP_INFO(get_logger(), "dry_run rest pose id19=%.0f id22=%.0f (not sent)", pose.id19,
                  pose.id22);
      return;
    }
    servo_pub_->publish(make_arm_command(pose));
  }

  // Stop the gait without sending Twist{}. gait 0 = stop_running; gait -2
  // = DEFAULT_POSE (stand). A zero cmd_vel would walk in place forever.
  void halt_legs() {
    walking_active_ = false;
    if (dry_run_) {
      RCLCPP_INFO(get_logger(), "dry_run halt_legs (Traveling gait 0 then -2, not sent)");
      return;
    }
    kinematics_msgs::msg::Traveling stop;
    stop.gait = 0;
    stop.time = 1.0f;
    stop.interrupt = true;
    traveling_pub_->publish(stop);

    kinematics_msgs::msg::Traveling stand;
    stand.gait = -2;
    stand.time = 1.0f;
    stand.interrupt = true;
    traveling_pub_->publish(stand);
  }

  void maybe_mark_ready() {
    if (hardware_ready_) {
      return;
    }
    if (!wait_for_ready_) {
      hardware_ready_ = true;
      halt_legs();
      send_rest_pose(rest_.duration_s);
      RCLCPP_INFO(get_logger(), "wait_for_ready=false; commanding rest pose");
      return;
    }
    const bool controller_ready = controller_client_->service_is_ready();
    const bool init_pose_ready = init_pose_client_->service_is_ready();
    if (controller_ready && init_pose_ready) {
      hardware_ready_ = true;
      halt_legs();
      send_rest_pose(rest_.duration_s);
      RCLCPP_INFO(get_logger(), "controller + init_pose ready; rest pose sent");
      return;
    }
    if ((now() - started_at_).seconds() > ready_timeout_seconds_) {
      hardware_ready_ = true;
      halt_legs();
      send_rest_pose(rest_.duration_s);
      RCLCPP_WARN(get_logger(),
                  "ready timeout (controller=%d init_pose=%d); continuing anyway "
                  "(slim bringup is usually already up)",
                  static_cast<int>(controller_ready), static_cast<int>(init_pose_ready));
    }
  }

  void on_start(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    halt_legs();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = false;
      target_ = TargetState{};
      lock_active_ = false;
      rest_sent_ = false;
      ema_ready_ = false;
      coast_det_.reset();
      latest_det_.reset();
      last_seen_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_fresh_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_walk_cmd_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      seen_streak_ = 0;
      reset_track_pending_ = true;
      pan_i_ = 0.0;
      gaze_id19_ = rest_.id19;
      gaze_id22_ = rest_.id22;
      set_phase_locked(Phase::Idle);
    }
    send_rest_pose(rest_.duration_s);
    response->success = true;
    response->message = "idle";
    RCLCPP_INFO(get_logger(), "start: reset to idle (rest pose, legs halted)");
  }

  void on_stop(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      lock_active_ = false;
      coast_det_.reset();
      latest_det_.reset();
      last_walk_cmd_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      set_phase_locked(Phase::Done);
    }
    halt_legs();
    send_rest_pose(rest_.duration_s);
    response->success = true;
    response->message = "done";
    RCLCPP_INFO(get_logger(), "stop: rest pose, legs halted, phase=done");
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    auto reject = [&](const std::string &reason) {
      result.successful = false;
      result.reason = reason;
    };

    for (const auto &p : params) {
      const std::string &name = p.get_name();
      if (name == "person_onnx" || name == "cascade_dir" || name == "lbp_dir" ||
          name == "dnn_cuda" || name == "person_imgsz" || name == "image_topic" ||
          name == "camera_info_topic" || name == "servo_topic" || name == "cmd_vel_topic") {
        reject(name + " cannot be changed at runtime");
        return result;
      }
      if (name == "ema_alpha") {
        const double v = p.as_double();
        if (v <= 0.0 || v > 1.0) {
          reject("ema_alpha must be in (0, 1]");
          return result;
        }
      }
      if (name == "walk_speed" || name == "walk_seconds" || name == "lock_seconds" ||
          name == "lost_timeout_seconds" || name == "walk_watchdog_seconds" ||
          name == "gaze_gain" || name == "max_step_pulses" || name == "deadband_rad" ||
          name == "person_conf" || name == "person_iou" || name == "max_jump_px") {
        if (p.as_double() < 0.0) {
          reject(name + " must be >= 0");
          return result;
        }
      }
      if (name == "confirm_frames" && p.as_int() < 1) {
        reject("confirm_frames must be >= 1");
        return result;
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    double new_pan_min = pulse_map_.pan_min;
    double new_pan_max = pulse_map_.pan_max;
    double new_tilt_min = pulse_map_.tilt_min;
    double new_tilt_max = pulse_map_.tilt_max;
    for (const auto &p : params) {
      if (p.get_name() == "pan_min") {
        new_pan_min = p.as_double();
      } else if (p.get_name() == "pan_max") {
        new_pan_max = p.as_double();
      } else if (p.get_name() == "tilt_min") {
        new_tilt_min = p.as_double();
      } else if (p.get_name() == "tilt_max") {
        new_tilt_max = p.as_double();
      }
    }
    if (new_pan_min >= new_pan_max) {
      reject("pan_min must be < pan_max");
      return result;
    }
    if (new_tilt_min >= new_tilt_max) {
      reject("tilt_min must be < tilt_max");
      return result;
    }
    for (const auto &p : params) {
      const std::string &name = p.get_name();
      if (name == "enable_walk") {
        enable_walk_ = p.as_bool();
      } else if (name == "dry_run") {
        dry_run_ = p.as_bool();
      } else if (name == "yaw_sign") {
        pulse_map_.yaw_sign = p.as_double();
      } else if (name == "pitch_sign") {
        pulse_map_.pitch_sign = p.as_double();
      } else if (name == "pan_min") {
        pulse_map_.pan_min = static_cast<float>(p.as_double());
      } else if (name == "pan_max") {
        pulse_map_.pan_max = static_cast<float>(p.as_double());
      } else if (name == "tilt_min") {
        pulse_map_.tilt_min = static_cast<float>(p.as_double());
      } else if (name == "tilt_max") {
        pulse_map_.tilt_max = static_cast<float>(p.as_double());
      } else if (name == "lock_seconds") {
        lock_seconds_ = p.as_double();
      } else if (name == "lost_timeout_seconds") {
        lost_timeout_seconds_ = p.as_double();
      } else if (name == "walk_seconds") {
        walk_seconds_ = p.as_double();
      } else if (name == "walk_speed") {
        walk_speed_ = p.as_double();
      } else if (name == "walk_watchdog_seconds") {
        walk_watchdog_seconds_ = p.as_double();
      } else if (name == "center_yaw_tol") {
        center_yaw_tol_ = p.as_double();
      } else if (name == "center_pitch_tol") {
        center_pitch_tol_ = p.as_double();
      } else if (name == "servo_duration") {
        servo_duration_ = p.as_double();
      } else if (name == "gaze_gain") {
        gaze_gain_ = p.as_double();
      } else if (name == "gaze_ki") {
        gaze_ki_ = p.as_double();
      } else if (name == "max_step_pulses") {
        max_step_pulses_ = p.as_double();
      } else if (name == "deadband_rad") {
        deadband_rad_ = p.as_double();
      } else if (name == "aim_u_offset_px") {
        aim_u_offset_px_ = p.as_double();
      } else if (name == "aim_v_offset_px") {
        aim_v_offset_px_ = p.as_double();
      } else if (name == "ema_alpha") {
        ema_alpha_ = p.as_double();
      } else if (name == "max_jump_px") {
        max_jump_px_ = p.as_double();
      } else if (name == "confirm_frames") {
        confirm_frames_ = static_cast<int>(p.as_int());
      } else if (name == "debug_max_width") {
        debug_max_width_ = static_cast<int>(p.as_int());
      } else if (name == "person_conf") {
        human_cfg_.person_conf = p.as_double();
      } else if (name == "person_iou") {
        human_cfg_.person_iou = p.as_double();
      } else if (name == "enable_face_refine") {
        human_cfg_.enable_face_refine = p.as_bool();
      } else if (name == "person_head_frac") {
        human_cfg_.person_head_frac = p.as_double();
      } else if (name == "max_box_frac") {
        human_cfg_.max_box_frac = p.as_double();
      } else if (name == "person_max_aspect") {
        human_cfg_.max_aspect = p.as_double();
      } else if (name == "track_gate_frac") {
        human_cfg_.track_gate_frac = p.as_double();
      } else if (name == "human_image_scale") {
        human_cfg_.image_scale = p.as_double();
      } else if (name == "human_scale_factor") {
        human_cfg_.scale_factor = p.as_double();
      } else if (name == "human_min_neighbors") {
        human_cfg_.min_neighbors = static_cast<int>(p.as_int());
      } else if (name == "min_face") {
        human_cfg_.min_face = static_cast<int>(p.as_int());
      }
    }
    return result;
  }

  void publish_debug_image(const cv::Mat &bgr,
                           const sensor_msgs::msg::Image::ConstSharedPtr &source) {
    cv_bridge::CvImage out;
    if (source) {
      out.header = source->header;
    } else {
      out.header.stamp = now();
    }
    out.header.frame_id = out.header.frame_id.empty() ? "depth_cam_rgb" : out.header.frame_id;
    out.encoding = "bgr8";
    if (debug_max_width_ > 0 && bgr.cols > debug_max_width_) {
      const double s = static_cast<double>(debug_max_width_) / static_cast<double>(bgr.cols);
      cv::resize(bgr, out.image, cv::Size(), s, s, cv::INTER_AREA);
    } else {
      out.image = bgr;
    }
    image_pub_->publish(*out.toImageMsg());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string servo_topic_;
  std::string cmd_vel_topic_;
  std::string controller_ready_service_;
  std::string init_pose_ready_service_;

  bool wait_for_ready_{true};
  bool image_qos_reliable_{false};
  bool enable_walk_{false};
  bool dry_run_{false};
  bool hardware_ready_{false};
  bool logged_intrinsics_{false};
  bool logged_fallback_intrinsics_{false};

  HumanDetectConfig human_cfg_;
  std::unique_ptr<HumanDetector> human_;
  PulseMapping pulse_map_;
  ArmPulses rest_;

  double lock_seconds_{2.0};
  double lost_timeout_seconds_{6.0};
  double walk_seconds_{3.0};
  double walk_speed_{0.05};
  double walk_watchdog_seconds_{0.3};
  double center_yaw_tol_{0.08};
  double center_pitch_tol_{0.20};
  double servo_duration_{0.08};
  double ready_timeout_seconds_{30.0};
  double gaze_gain_{0.18};
  double gaze_ki_{0.0};
  double max_step_pulses_{8.0};
  double deadband_rad_{0.06};
  double aim_u_offset_px_{0.0};
  double aim_v_offset_px_{0.0};
  double pan_i_{0.0};
  double ema_alpha_{0.30};
  double max_jump_px_{180.0};
  int debug_max_width_{640};
  int confirm_frames_{3};
  float gaze_id19_{500.0f};
  float gaze_id22_{150.0f};
  double ema_u_{0.0};
  double ema_v_{0.0};
  bool ema_ready_{false};
  int seen_streak_{0};

  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::CallbackGroup::SharedPtr detect_cb_group_;
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
  rclcpp::CallbackGroup::SharedPtr watchdog_cb_group_;
  rclcpp::Publisher<servo_controller_msgs::msg::ServosPosition>::SharedPtr servo_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<kinematics_msgs::msg::Traveling>::SharedPtr traveling_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr controller_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr init_pose_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
  rclcpp::TimerBase::SharedPtr detect_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::mutex mutex_;
  Phase phase_{Phase::Idle};
  TargetState target_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  std::optional<PersonDetection> latest_det_;
  GazeAngles latest_angles_;
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fresh_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_walk_cmd_at_{0, 0, RCL_ROS_TIME};
  std::optional<PersonDetection> coast_det_;
  rclcpp::Time lock_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time walk_started_at_{0, 0, RCL_ROS_TIME};
  bool lock_active_{false};
  bool rest_sent_{false};
  bool reset_track_pending_{false};
  bool stopping_{false};
  bool walking_active_{false};
};

}  // namespace proud_up

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<proud_up::FollowTheCatNode>();
  std::weak_ptr<proud_up::FollowTheCatNode> weak = node;
  rclcpp::on_shutdown([weak]() {
    if (auto n = weak.lock()) {
      n->emergency_stop();
    }
  });

  // MultiThreadedExecutor: image, detect, control, and watchdog groups can run
  // on different threads. A single-threaded executor would serialise YOLO
  // behind servo commands and make the walk watchdog late.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  node->emergency_stop();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
