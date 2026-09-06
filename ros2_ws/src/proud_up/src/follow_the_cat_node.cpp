// follow_the_cat_node — Week 2 on Masha (Jetson Orin NX, ROS 2 Humble).
//
// Detect a person (YOLOv8n COCO, LBP face inside the box) in the Aurora RGB
// image, pan/tilt the arm-mounted camera onto the head (servos 19 and 22),
// then optionally walk 0.05 m/s for at most 3 s.
// Math lives in follow_the_cat.hpp (no rclcpp). This file is the ROS wiring:
// best-effort image sub, 10 Hz timer, mutex, services, servo + cmd_vel pubs.
//
// Threads: image/camera_info callbacks only store shared_ptrs under mutex_.
// The timer copies those pointers, drops the lock, then runs OpenCV / Eigen.
// Two callback groups + MultiThreadedExecutor make that overlap real.
//
// Gaze is incremental (integrate_gaze), not rest+angle: commanding rest+yaw
// every tick hunts, because a centered card would snap the arm back to rest.
// White walls are dropped (border-touching blobs); they are not a "card".
//
// Safety: enable_walk defaults false. Do NOT publish a zero Twist to
// /controller/cmd_vel — move_controller still starts CmdVelGenerator at vx=0,
// which is a forever gait with no translation ("walks in place"). To stop
// legs, publish Traveling gait=0 (stop_running) then gait=-2 (DEFAULT_POSE).
// Only arm ids 19–24.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kinematics_msgs/msg/traveling.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "proud_up/follow_the_cat.hpp"
#include "proud_up/pose_commander.hpp"

using namespace std::chrono_literals;

namespace proud_up {
namespace {

// IDLE: rest pose, wait for a blob. Never touch cmd_vel here.
// TRACKING: 10 Hz pan/tilt. Walk is gated on ~2 s of continuous lock near center.
// MOVING: linear.x = walk_speed on /controller/cmd_vel for at most walk_seconds.
// DONE: hold gaze; halt legs via Traveling, not a zero Twist.
// LOST: blob gone; next tick returns to IDLE (aborts a walk if one was running).
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

// Latest blob as seen by the timer, plus the ROS stamp of that image.
// Guarded by the node mutex together with the image / camera_info pointers.
// The image callback does NOT write this — it only stores the Image pointer
// (cheap shared_ptr copy). Detection runs on the timer thread after the lock
// is released, then the result is written back under the lock.
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
    target_mode_ = declare_parameter<std::string>("target", "human");

    HumanDetectConfig human_cfg;
    human_cfg.cascade_dir = declare_parameter<std::string>(
        "cascade_dir", "/usr/share/opencv4/haarcascades");
    human_cfg.lbp_dir = declare_parameter<std::string>(
        "lbp_dir", "/usr/share/opencv4/lbpcascades");
    human_cfg.image_scale = declare_parameter<double>("human_image_scale", 0.45);
    human_cfg.scale_factor = declare_parameter<double>("human_scale_factor", 1.2);
    human_cfg.min_neighbors = declare_parameter<int>("human_min_neighbors", 3);
    human_cfg.min_face = declare_parameter<int>("min_face", 24);
    human_cfg.min_upper = declare_parameter<int>("min_upper", 50);
    human_cfg.min_full = declare_parameter<int>("min_full", 60);
    human_cfg.enable_upper = declare_parameter<bool>("enable_upper_body", false);
    human_cfg.enable_full = declare_parameter<bool>("enable_full_body", false);
    human_cfg.person_conf = declare_parameter<double>("person_conf", 0.45);
    human_cfg.person_iou = declare_parameter<double>("person_iou", 0.45);
    human_cfg.person_imgsz = declare_parameter<int>("person_imgsz", 320);
    human_cfg.dnn_cuda = declare_parameter<bool>("dnn_cuda", true);
    human_cfg.enable_face_refine = declare_parameter<bool>("enable_face_refine", false);
    human_cfg.person_head_frac = declare_parameter<double>("person_head_frac", 0.45);
    human_cfg.max_box_frac = declare_parameter<double>("max_box_frac", 0.40);
    human_cfg.max_aspect = declare_parameter<double>("person_max_aspect", 1.05);
    human_cfg.track_gate_frac = declare_parameter<double>("track_gate_frac", 0.22);
    human_cfg.person_onnx = declare_parameter<std::string>("person_onnx", "");
    if (human_cfg.person_onnx.empty()) {
      try {
        human_cfg.person_onnx =
            ament_index_cpp::get_package_share_directory("proud_up") + "/models/yolov8n.onnx";
      } catch (const std::exception &) {
        human_cfg.person_onnx.clear();
      }
    }
    if (!human_cfg.person_onnx.empty()) {
      std::ifstream probe(human_cfg.person_onnx);
      if (!probe.good()) {
        RCLCPP_ERROR(get_logger(), "person ONNX missing: %s", human_cfg.person_onnx.c_str());
        human_cfg.person_onnx.clear();
      }
    }
    human_ = std::make_unique<HumanDetector>(human_cfg);
    if (target_mode_ == "human" && !human_->ok()) {
      RCLCPP_ERROR(get_logger(),
                   "no person detector (cascade %s / %s, onnx %s) — gaze will miss",
                   human_cfg.lbp_dir.c_str(), human_cfg.cascade_dir.c_str(),
                   human_cfg.person_onnx.c_str());
    } else if (target_mode_ == "human") {
      RCLCPP_INFO(get_logger(), "person detector onnx=%s cuda=%s face=%s",
                  human_->person_ok() ? human_cfg.person_onnx.c_str() : "off",
                  human_->using_cuda() ? "yes" : "no",
                  human_cfg.cascade_dir.empty() ? "off" : "lbp");
    }

    detect_cfg_.threshold = declare_parameter<int>("threshold", 90);
    detect_cfg_.loose_threshold = declare_parameter<int>("loose_threshold", 130);
    detect_cfg_.min_area = declare_parameter<double>("min_area", 800.0);
    detect_cfg_.max_area = declare_parameter<double>("max_area", 80000.0);
    detect_cfg_.max_area_frac = declare_parameter<double>("max_area_frac", 0.22);
    detect_cfg_.min_aspect = declare_parameter<double>("min_aspect", 0.28);
    detect_cfg_.max_aspect = declare_parameter<double>("max_aspect", 0.85);
    detect_cfg_.min_solidity = declare_parameter<double>("min_solidity", 0.55);
    detect_cfg_.blur_ksize = declare_parameter<int>("blur_ksize", 5);
    detect_cfg_.morph_ksize = declare_parameter<int>("morph_ksize", 5);
    detect_cfg_.border_margin = declare_parameter<int>("border_margin", 4);
    detect_cfg_.reject_border = declare_parameter<bool>("reject_border", true);
    detect_cfg_.require_quad = declare_parameter<bool>("require_quad", true);
    detect_cfg_.require_portrait = declare_parameter<bool>("require_portrait", true);

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

    gaze_id19_ = rest_.id19;
    gaze_id22_ = rest_.id22;

    // Two mutually exclusive groups so the image callback and the 10 Hz timer
    // can actually overlap on a MultiThreadedExecutor. The default group is
    // mutually exclusive for the *whole node*, which would serialize them and
    // hide the race the mutex is there to prevent.
    image_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    servo_pub_ = create_publisher<servo_controller_msgs::msg::ServosPosition>(servo_topic_, 1);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
    traveling_pub_ = create_publisher<kinematics_msgs::msg::Traveling>("/controller/traveling", 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_result", 1);

    // Aurora publishes best-effort. A default *reliable* subscription on
    // Humble often sees zero frames — proud_up_node already uses KeepLast(5)
    // + best_effort for this reason. camera_info is typically reliable; we
    // still match the image QoS so a best-effort info topic would work too.
    rclcpp::QoS image_qos(rclcpp::KeepLast(5));
    if (image_qos_reliable_) {
      image_qos.reliable();
    } else {
      image_qos.best_effort();
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

    timer_ = create_wall_timer(50ms, [this]() { tick(); }, control_cb_group_);
    started_at_ = now();
    phase_started_at_ = started_at_;

    RCLCPP_INFO(get_logger(),
                "follow_the_cat target=%s image=%s info=%s servo=%s cmd_vel=%s "
                "enable_walk=%s dry_run=%s yaw_sign=%.1f pitch_sign=%.1f "
                "(tilt is servo 22, not 23)",
                target_mode_.c_str(), image_topic_.c_str(), camera_info_topic_.c_str(),
                servo_topic_.c_str(), cmd_vel_topic_.c_str(),
                enable_walk_ ? "true" : "false", dry_run_ ? "true" : "false",
                pulse_map_.yaw_sign, pulse_map_.pitch_sign);
    if (!enable_walk_) {
      RCLCPP_INFO(get_logger(),
                  "walk is OFF (enable_walk:=false). First bring-up is gaze-only. "
                  "Flip yaw_sign/pitch_sign on hardware before enabling walk.");
    }
  }

  ~FollowTheCatNode() override { emergency_stop(); }

  // Called from main after spin, and from the destructor / SIGINT path.
  // Always publishes a zero Twist so the hexapod does not keep the last
  // 0.05 m/s command if the node dies mid-walk.
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

  // Callback thread: store the shared_ptr (refcount++, no pixel copy) and
  // return. Do not call OpenCV here — a slow findContours would stall DDS.
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_image_ = std::move(msg);
  }

  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_info_ = std::move(msg);
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
      last_seen_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_fresh_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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
      set_phase_locked(Phase::Done);
    }
    halt_legs();
    send_rest_pose(rest_.duration_s);
    response->success = true;
    response->message = "done";
    RCLCPP_INFO(get_logger(), "stop: rest pose, legs halted, phase=done");
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
    }
  }

  void tick() {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    bool stopping;
    DetectionConfig detect_cfg;
    PulseMapping pulse_map;
    bool enable_walk;
    bool dry_run;
    bool reset_track = false;
    {
      // Copy pointers / small values, then drop the lock BEFORE OpenCV.
      // Holding it across toCvCopy would stall the callback thread (and
      // therefore the DDS executor) for every frame.
      std::lock_guard<std::mutex> lock(mutex_);
      image = latest_image_;
      info = latest_info_;
      stopping = stopping_;
      detect_cfg = detect_cfg_;
      pulse_map = pulse_map_;
      enable_walk = enable_walk_;
      dry_run = dry_run_;
      reset_track = reset_track_pending_;
      reset_track_pending_ = false;
    }

    if (stopping) {
      return;
    }

    maybe_mark_ready();

    if (reset_track && human_) {
      human_->reset_track();
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

    std::optional<CardDetection> det;
    DetectionStats det_stats;
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
        if (target_mode_ == "human") {
          det = human_->detect(annotated);
          const double ms = human_->last_detect_ms();
          if (ms > 80.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "person detect took %.0f ms — loop is lagging", ms);
          }
        } else {
          det = detect_black_rectangle(annotated, detect_cfg, &det_stats);
        }
        {
          // Hold the last box for lost_timeout_seconds so a single YOLO miss
          // (you turn, a frame is dark) does not snap idle in 0.8 s.
          std::lock_guard<std::mutex> lock(mutex_);
          if (det) {
            coast_det_ = det;
            last_fresh_at_ = now();
          } else if (coast_det_ && last_fresh_at_.nanoseconds() != 0 &&
                     (now() - last_fresh_at_).seconds() <= lost_timeout_seconds_) {
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
            // Drive the aim point to the *visual* centre of the frame (magenta
            // cross), not K.cx — that is what "centre me in the picture" means.
            CameraIntrinsics aim = K;
            aim.cx = 0.5 * static_cast<double>(annotated.cols) + aim_u_offset_px_;
            aim.cy = 0.5 * static_cast<double>(annotated.rows) + aim_v_offset_px_;
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

    geometry_msgs::msg::Twist twist;  // only published when linear.x != 0
    bool publish_twist = false;
    bool publish_servos = false;
    bool halt_after_unlock = false;
    ArmPulses arm = rest_;
    double servo_dt = servo_duration_;
    Phase overlay_phase = Phase::Idle;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        return;
      }

      target_.seen = static_cast<bool>(det);
      if (det) {
        target_.u = det->center.u;
        target_.v = det->center.v;
        target_.stamp = image_stamp;
        if (det->label == nullptr || std::strcmp(det->label, "coast") != 0) {
          last_seen_at_ = now();
        }
      }

      if (!hardware_ready_) {
        overlay_phase = phase_;
      } else {
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
            // Hold the last pan/tilt. Do not snap back to rest — that yank is
            // what "looks around randomly" felt like when YOLO blinked.
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
        overlay_phase = phase_;
      }
    }

    const char *phase_str = phase_name(overlay_phase);
    if (!annotated.empty()) {
      char err_note[128];
      if (det) {
        const double du = det->center.u - 0.5 * annotated.cols - aim_u_offset_px_;
        std::snprintf(err_note, sizeof(err_note),
                      "du=%+.0f px  yaw=%+.1f deg  det=%.0fms", du,
                      angles.yaw * 180.0 / kPi,
                      human_ ? human_->last_detect_ms() : 0.0);
      } else {
        std::snprintf(err_note, sizeof(err_note), "stand in front of Masha, face the camera");
      }
      draw_debug_overlay(annotated, det, phase_str, det ? &angles : nullptr, &det_stats,
                         err_note);
      publish_debug_image(annotated, image);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
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
    // Never publish a zero Twist. vx=0 still starts CmdVelGenerator forever
    // (in-place stepping). Only a real forward command is allowed here.
    if (publish_twist && twist.linear.x != 0.0) {
      if (dry_run) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "dry_run walk linear.x=%.3f (not sent)", twist.linear.x);
      } else {
        cmd_vel_pub_->publish(twist);
      }
    }
    if (halt_after_unlock) {
      halt_legs();
    }
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

  void tick_idle_locked(const std::optional<CardDetection> &det, const GazeAngles &angles,
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
    // Wait a few consistent frames before the first servo step so a single
    // wall-glint cannot yank the arm.
    if (seen_streak_ >= confirm_frames_) {
      publish_servos = apply_gaze(arm, servo_dt, angles, pulse_map);
      set_phase_locked(Phase::Tracking);
    }
  }

  void tick_tracking_locked(const std::optional<CardDetection> &det, const GazeAngles &angles,
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
    const bool coasting = det->label != nullptr && std::strcmp(det->label, "coast") == 0;
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
        // Walk is a one-shot 15 cm-class step, not a chase. enable_walk defaults
        // false so Day 5 can verify pan/tilt signs with the card held still.
        set_phase_locked(Phase::Moving);
      }
    } else {
      lock_active_ = false;
    }
  }

  void tick_moving_locked(const std::optional<CardDetection> &det, const GazeAngles &angles,
                          const PulseMapping &pulse_map, ArmPulses &arm, double &servo_dt,
                          bool &publish_servos, geometry_msgs::msg::Twist &twist,
                          bool &publish_twist, bool &halt_after_unlock) {
    if (!det) {
      // Abort immediately if the blob is lost — do not keep walking blind.
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

  // Returns true if the commanded pulses actually moved. Re-sending the same
  // 80 ms servo packet at 20 Hz makes the arm twitch even when the error is
  // inside the deadband.
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
    arm.id19 = next.id19;  // pan, joint1. Clamped 200–800 inside integrate_gaze.
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

  // Stop the gait without sending Twist{}. gait 0 = stop_running; gait -2 =
  // DEFAULT_POSE (stand). A zero cmd_vel would walk in place forever.
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

  DetectionConfig detect_cfg_;
  PulseMapping pulse_map_;
  ArmPulses rest_;
  std::string target_mode_{"human"};
  std::unique_ptr<HumanDetector> human_;

  double lock_seconds_{2.0};
  double lost_timeout_seconds_{2.5};
  double walk_seconds_{3.0};
  double walk_speed_{0.05};
  double center_yaw_tol_{0.08};
  double center_pitch_tol_{0.20};
  double servo_duration_{0.12};
  double ready_timeout_seconds_{30.0};
  double gaze_gain_{0.45};
  double gaze_ki_{0.35};
  double max_step_pulses_{28.0};
  double deadband_rad_{0.02};
  double aim_u_offset_px_{0.0};
  double aim_v_offset_px_{0.0};
  double pan_i_{0.0};
  double ema_alpha_{0.35};
  double max_jump_px_{400.0};
  int debug_max_width_{320};
  int confirm_frames_{1};
  float gaze_id19_{500.0f};
  float gaze_id22_{150.0f};
  double ema_u_{0.0};
  double ema_v_{0.0};
  bool ema_ready_{false};
  int seen_streak_{0};

  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
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
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  Phase phase_{Phase::Idle};
  TargetState target_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fresh_at_{0, 0, RCL_ROS_TIME};
  std::optional<CardDetection> coast_det_;
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

  // MultiThreadedExecutor: image callback group and timer group can run at the
  // same time. The mutex in the node is what makes that safe.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  node->emergency_stop();
  rclcpp::shutdown();
  return 0;
}
