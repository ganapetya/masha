// masha_hunter_node — spider hunter on Masha (Jetson Orin NX, ROS 2 Humble).
//
// Policy lives in hunter.hpp (no ROS). This file is the wiring:
// sensors, TF, arm pan, Traveling halt, aplay worker, overlay.
//
//   IDLE --~/start--> HUNT --hit--> NAME --wav--> FOLLOW (≤60 s)
//                         ^                         |
//                         +-- lost / timeout / LiDAR +
//
// Do not launch follow_the_cat_node, masha_interaction FOLLOW, lidar_app,
// Nav2, or analog joystick at the same time. Two writers on
// /controller/cmd_vel and on servos 19/22 fight.
//
// Threads: image/scan callbacks only store pointers. Detect (YOLO) and
// control (PID + publish) run on timers. Never OpenCV or aplay on DDS.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <spawn.h>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kinematics_msgs/msg/traveling.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "proud_up/follow_the_cat.hpp"
#include "proud_up/hunter.hpp"
#include "proud_up/pose_commander.hpp"
#include "proud_up/target_source.hpp"

extern char **environ;

using namespace std::chrono_literals;

namespace proud_up {
namespace {

pid_t spawn_argv(const std::vector<std::string> &args) {
  if (args.empty()) {
    return -1;
  }
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto &a : args) {
    argv.push_back(const_cast<char *>(a.c_str()));
  }
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attr;
  posix_spawn_file_actions_init(&actions);
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attr, 0);
  pid_t pid = -1;
  const int rc = posix_spawnp(&pid, argv[0], &actions, &attr, argv.data(), environ);
  posix_spawnattr_destroy(&attr);
  posix_spawn_file_actions_destroy(&actions);
  if (rc != 0) {
    return -1;
  }
  return pid;
}

void kill_group(pid_t pid) {
  if (pid <= 0) {
    return;
  }
  if (killpg(pid, SIGTERM) != 0) {
    kill(pid, SIGTERM);
  }
  std::this_thread::sleep_for(80ms);
  if (killpg(pid, 0) == 0 || kill(pid, 0) == 0) {
    killpg(pid, SIGKILL);
    kill(pid, SIGKILL);
  }
}

bool file_exists(const std::string &path) {
  struct stat st {};
  return !path.empty() && stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

class WavPlayer {
 public:
  ~WavPlayer() { stop(); }

  void play_once(const std::string &wav, double timeout_s) {
    stop();
    if (!file_exists(wav)) {
      finished_.store(true);
      return;
    }
    stop_.store(false);
    finished_.store(false);
    running_.store(true);
    wav_ = wav;
    timeout_s_ = timeout_s;
    th_ = std::thread([this]() { worker(); });
  }

  void stop() {
    stop_.store(true);
    const pid_t pid = pid_.load();
    if (pid > 0) {
      kill_group(pid);
    }
    if (th_.joinable()) {
      th_.join();
    }
    pid_.store(0);
    running_.store(false);
  }

  bool running() const { return running_.load(); }
  bool finished() const { return finished_.load(); }

 private:
  void worker() {
    const pid_t pid = spawn_argv({"/usr/bin/aplay", "-q", "-D", "pulse", wav_});
    if (pid <= 0) {
      running_.store(false);
      finished_.store(true);
      return;
    }
    pid_.store(pid);
    const auto start = std::chrono::steady_clock::now();
    while (!stop_.load()) {
      int status = 0;
      const pid_t r = waitpid(pid, &status, WNOHANG);
      if (r == pid) {
        break;
      }
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (timeout_s_ > 0.0 && elapsed >= timeout_s_) {
        kill_group(pid);
        waitpid(pid, &status, 0);
        break;
      }
      std::this_thread::sleep_for(20ms);
    }
    pid_.store(0);
    running_.store(false);
    finished_.store(true);
  }

  std::thread th_;
  std::string wav_;
  double timeout_s_{3.0};
  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<pid_t> pid_{0};
};

double yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

class MashaHunterNode : public rclcpp::Node {
 public:
  MashaHunterNode() : Node("masha_hunter_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    image_topic_ = declare_parameter<std::string>("image_topic", "/depth_cam/rgb/image_raw");
    camera_info_topic_ =
        declare_parameter<std::string>("camera_info_topic", "/depth_cam/rgb/camera_info");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_cam/depth/image_raw");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    apriltag_topic_ =
        declare_parameter<std::string>("apriltag_topic", "/apriltag/apriltag_detections");
    servo_topic_ = declare_parameter<std::string>("servo_topic", "servo_controller");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/controller/cmd_vel");
    traveling_topic_ = declare_parameter<std::string>("traveling_topic", "/controller/traveling");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "rgb_camera_link");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    saveli_tag_frame_ = declare_parameter<std::string>("saveli_tag_frame", "saveli_tag");
    voice_dir_ = declare_parameter<std::string>("voice_dir", "");

    enabled_targets_ = declare_parameter<std::vector<std::string>>("enabled_targets", {"saveli"});
    saveli_tag_id_ = declare_parameter<int>("saveli_tag_id", 0);
    dry_run_ = declare_parameter<bool>("dry_run", false);

    HunterConfig cfg;
    cfg.enable_walk = declare_parameter<bool>("enable_walk", false);
    cfg.enable_crab = declare_parameter<bool>("enable_crab", false);
    cfg.enable_wander = declare_parameter<bool>("enable_wander", false);
    cfg.r_target = declare_parameter<double>("r_target", 0.80);
    cfg.d_stop = declare_parameter<double>("d_stop", 0.55);
    cfg.d_go = declare_parameter<double>("d_go", 0.70);
    cfg.lost_timeout = declare_parameter<double>("lost_timeout", 0.5);
    cfg.follow_max_s = declare_parameter<double>("follow_max_s", 60.0);
    cfg.search_timeout_s = declare_parameter<double>("search_timeout_s", 20.0);
    cfg.vx_max = declare_parameter<double>("vx_max", 0.05);
    cfg.vy_max = declare_parameter<double>("vy_max", 0.04);
    cfg.wz_max = declare_parameter<double>("wz_max", 0.30);
    cfg.kp_x = declare_parameter<double>("kp_x", 0.35);
    cfg.kd_x = declare_parameter<double>("kd_x", 0.05);
    cfg.kp_y = declare_parameter<double>("kp_y", 0.50);
    cfg.kd_y = declare_parameter<double>("kd_y", 0.05);
    cfg.kp_th = declare_parameter<double>("kp_th", 1.20);
    cfg.kd_th = declare_parameter<double>("kd_th", 0.10);
    cfg.wander_vx = declare_parameter<double>("wander_vx", 0.03);
    cfg.wander_wz = declare_parameter<double>("wander_wz", 0.25);
    cfg.lidar_x = declare_parameter<double>("lidar_x", 0.102);
    cfg.lidar_y = declare_parameter<double>("lidar_y", 0.0);
    walk_watchdog_s_ = declare_parameter<double>("walk_watchdog", 0.3);
    hunter_ = Hunter(cfg);

    rest_.id19 = static_cast<float>(declare_parameter<double>("arm_id19", 500.0));
    rest_.id20 = static_cast<float>(declare_parameter<double>("arm_id20", 810.0));
    rest_.id21 = static_cast<float>(declare_parameter<double>("arm_id21", 180.0));
    rest_.id22 = static_cast<float>(declare_parameter<double>("arm_id22", 150.0));
    rest_.id23 = static_cast<float>(declare_parameter<double>("arm_id23", 500.0));
    rest_.id24 = static_cast<float>(declare_parameter<double>("arm_id24", 500.0));
    rest_.duration_s = declare_parameter<double>("arm_duration", 1.0);
    pan_min_ = static_cast<float>(declare_parameter<double>("pan_min", 200.0));
    pan_max_ = static_cast<float>(declare_parameter<double>("pan_max", 800.0));
    pan_period_s_ = declare_parameter<double>("pan_period_s", 10.0);
    follow_gait_select_ = declare_parameter<int>("follow_gait_select", 15);
    cmd_height_ = static_cast<float>(declare_parameter<double>("cmd_height", 25.0));
    cmd_period_ = static_cast<float>(declare_parameter<double>("cmd_period", 0.70));
    gaze_gain_ = declare_parameter<double>("gaze_gain", 0.10);
    max_step_pulses_ = declare_parameter<double>("max_step_pulses", 8.0);
    deadband_rad_ = declare_parameter<double>("deadband_rad", 0.06);
    pulse_map_.yaw_sign = declare_parameter<double>("yaw_sign", -1.0);
    pulse_map_.pitch_sign = declare_parameter<double>("pitch_sign", 1.0);
    pulse_map_.pan_min = pan_min_;
    pulse_map_.pan_max = pan_max_;
    pulse_map_.pan_rest = rest_.id19;
    pulse_map_.tilt_rest = rest_.id22;
    gaze_id19_ = rest_.id19;
    gaze_id22_ = rest_.id22;

    if (voice_dir_.empty()) {
      try {
        const auto xf = ament_index_cpp::get_package_share_directory("xf_mic_asr_offline");
        voice_dir_ = xf + "/feedback_voice/english";
      } catch (const std::exception &) {
        voice_dir_ = "/home/ubuntu/ros2_ws/src/xf_mic_asr_offline/feedback_voice/english";
      }
    }

    saveli_wav_ = declare_parameter<std::string>("saveli_wav", voice_dir_ + "/saveli.wav");
    cat_wav_ = declare_parameter<std::string>("cat_wav", voice_dir_ + "/cat.wav");

    saveli_ = std::make_unique<SaveliSource>(saveli_tag_id_, saveli_wav_);
    saveli_->set_enabled(target_enabled("saveli"));

    HumanDetectConfig cat_cfg;
    cat_cfg.coco_class = 15;
    cat_cfg.require_person_shape = false;
    cat_cfg.enable_face_fallback = false;
    cat_cfg.person_head_frac = 0.50;
    cat_cfg.person_imgsz = declare_parameter<int>("person_imgsz", 320);
    cat_cfg.person_conf = declare_parameter<double>("person_conf", 0.45);
    cat_cfg.dnn_cuda = declare_parameter<bool>("dnn_cuda", true);
    cat_cfg.person_onnx = declare_parameter<std::string>("person_onnx", "");
    if (cat_cfg.person_onnx.empty()) {
      try {
        cat_cfg.person_onnx =
            ament_index_cpp::get_package_share_directory("proud_up") + "/models/yolov8n.onnx";
      } catch (const std::exception &) {
        cat_cfg.person_onnx = "/home/ubuntu/ros2_ws/src/proud_up/models/yolov8n.onnx";
      }
    }
    if (target_enabled("cat")) {
      cat_ = std::make_unique<CatSource>(cat_cfg, cat_wav_);
      cat_->set_enabled(true);
    }

    image_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    detect_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    servo_pub_ = create_publisher<servo_controller_msgs::msg::ServosPosition>(servo_topic_, 1);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
    traveling_pub_ = create_publisher<kinematics_msgs::msg::Traveling>(traveling_topic_, 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_result", 1);

    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    rclcpp::SubscriptionOptions image_opts;
    image_opts.callback_group = image_cb_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(std::move(msg)); },
        image_opts);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, sensor_qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { on_info(std::move(msg)); },
        image_opts);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_depth(std::move(msg)); },
        image_opts);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, sensor_qos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(std::move(msg)); },
        image_opts);
    // Saveli pose is TF child saveli_tag. Do not subscribe to apriltag_msgs:
    // Humble and the Jetson overlay ship different AprilTagDetection layouts;
    // deserializing the wrong one SIGSEGVs the node (start service never appears).

    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/start",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { on_start(response); },
        rmw_qos_profile_services_default, service_cb_group_);
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { on_stop(response); },
        rmw_qos_profile_services_default, service_cb_group_);

    detect_timer_ = create_wall_timer(50ms, [this]() { detect_tick(); }, detect_cb_group_);
    control_timer_ = create_wall_timer(50ms, [this]() { control_tick(); }, control_cb_group_);
    watchdog_timer_ = create_wall_timer(100ms, [this]() { watchdog_tick(); }, watchdog_cb_group_);

    RCLCPP_INFO(get_logger(),
                "masha_hunter enable_walk=%s enable_crab=%s enable_wander=%s "
                "targets=%zu gait_select=%d saveli_frame=%s (TF, not %s). "
                "Call ~/start to hunt. Zero Twist is not a halt.",
                hunter_.config().enable_walk ? "true" : "false",
                hunter_.config().enable_crab ? "true" : "false",
                hunter_.config().enable_wander ? "true" : "false", enabled_targets_.size(),
                follow_gait_select_, saveli_tag_frame_.c_str(), apriltag_topic_.c_str());
  }

  ~MashaHunterNode() override { emergency_stop(); }

  void emergency_stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      hunter_.stop();
      walking_active_ = false;
    }
    player_.stop();
    halt_legs();
  }

 private:
  bool target_enabled(const std::string &id) const {
    return std::find(enabled_targets_.begin(), enabled_targets_.end(), id) !=
           enabled_targets_.end();
  }

  rclcpp::Time now() { return get_clock()->now(); }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_image_ = std::move(msg);
  }
  void on_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_info_ = std::move(msg);
  }
  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_depth_ = std::move(msg);
  }
  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = std::move(msg);
  }

  void detect_tick() {
    try {
      detect_tick_inner();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "detect_tick: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "detect_tick: unknown exception");
    }
  }

  void detect_tick_inner() {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    bool run_cat = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        return;
      }
      image = latest_image_;
      depth = latest_depth_;
      info = latest_info_;
      run_cat = cat_ && cat_->enabled();
    }
    if (!run_cat || !image) {
      return;
    }
    cv_bridge::CvImagePtr cv;
    try {
      cv = cv_bridge::toCvCopy(image, "bgr8");
    } catch (const cv_bridge::Exception &) {
      return;
    }
    cv::Mat depth_mm;
    if (depth) {
      try {
        auto d = cv_bridge::toCvCopy(depth);
        if (d->image.type() == CV_16UC1) {
          depth_mm = d->image;
        }
      } catch (const cv_bridge::Exception &) {
      }
    }
    DetectInput in;
    in.bgr = &cv->image;
    in.depth_mm = depth_mm.empty() ? nullptr : &depth_mm;
    if (info && info->k.size() >= 9) {
      in.K = from_k_matrix(static_cast<int>(info->width), static_cast<int>(info->height),
                           info->k.data());
    }
    auto hit = cat_->detect(in);
    std::lock_guard<std::mutex> lock(mutex_);
    cat_hit_ = std::move(hit);
  }

  void fill_cat_base_pose(TargetHit &hit, sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
    if (!hit.pose.has_pixel || hit.range_m <= 0.05) {
      return;
    }
    try {
      if (!tf_buffer_._frameExists(base_frame_) || !tf_buffer_._frameExists(camera_frame_)) {
        return;
      }
      if (!tf_buffer_.canTransform(base_frame_, camera_frame_, tf2::TimePointZero)) {
        return;
      }
      auto tf = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
      CameraIntrinsics K = fallback_intrinsics();
      if (info && info->k.size() >= 9) {
        K = from_k_matrix(static_cast<int>(info->width), static_cast<int>(info->height),
                          info->k.data());
      }
      const Eigen::Vector3d ray = pixel_to_ray(hit.pose.u, hit.pose.v, K);
      geometry_msgs::msg::PointStamped cam;
      cam.header.frame_id = camera_frame_;
      cam.point.x = ray.x() * hit.range_m;
      cam.point.y = ray.y() * hit.range_m;
      cam.point.z = ray.z() * hit.range_m;
      geometry_msgs::msg::PointStamped base;
      tf2::doTransform(cam, base, tf);
      hit.pose.x = base.point.x;
      hit.pose.y = base.point.y;
      hit.pose_in_base = true;
    } catch (const tf2::TransformException &) {
    }
  }

  std::string saveli_tf_frame() const {
    // canTransform() prints to stderr if the frame was never published.
    // Check existence first. Fallback names: yaml tag_frames not applied
    // → AprilTagNode uses family:id (tag36h11:0).
    if (tf_buffer_._frameExists(saveli_tag_frame_)) {
      return saveli_tag_frame_;
    }
    const std::string family_id = "tag36h11:" + std::to_string(saveli_tag_id_);
    if (tf_buffer_._frameExists(family_id)) {
      return family_id;
    }
    const std::string short_id = "36h11:" + std::to_string(saveli_tag_id_);
    if (tf_buffer_._frameExists(short_id)) {
      return short_id;
    }
    return {};
  }

  std::optional<TargetHit> saveli_from_tf() {
    if (!saveli_ || !saveli_->enabled()) {
      return std::nullopt;
    }
    if (!tf_buffer_._frameExists(base_frame_)) {
      return std::nullopt;
    }
    const std::string tag_frame = saveli_tf_frame();
    if (tag_frame.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no Saveli TF yet (want %s or tag36h11:%d). Tag not in view, or "
                           "not 36h11 id %d.",
                           saveli_tag_frame_.c_str(), saveli_tag_id_, saveli_tag_id_);
      return std::nullopt;
    }
    DetectInput in;
    in.tag_id = saveli_tag_id_;
    try {
      if (!tf_buffer_.canTransform(base_frame_, tag_frame, tf2::TimePointZero)) {
        return std::nullopt;
      }
      auto tf = tf_buffer_.lookupTransform(base_frame_, tag_frame, tf2::TimePointZero);
      const double age = (now() - rclcpp::Time(tf.header.stamp)).seconds();
      if (age > hunter_.config().lost_timeout) {
        return std::nullopt;
      }
      in.have_saveli_pose = true;
      in.saveli_pose.x = tf.transform.translation.x;
      in.saveli_pose.y = tf.transform.translation.y;
      in.saveli_pose.yaw = yaw_from_quat(tf.transform.rotation);
      in.saveli_pose.has_pixel = false;
    } catch (const tf2::TransformException &) {
      // No tag TF yet (or base_link chain not up). Hunt keeps panning.
    } catch (const std::exception &) {
      return std::nullopt;
    }
    if (!in.have_saveli_pose) {
      return std::nullopt;
    }
    return saveli_->detect(in);
  }

  void control_tick() {
    try {
      control_tick_inner();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "control_tick: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "control_tick: unknown exception");
    }
  }

  void control_tick_inner() {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    sensor_msgs::msg::Image::ConstSharedPtr image;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    std::optional<TargetHit> cat_hit;
    HunterPhase prev;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        return;
      }
      scan = latest_scan_;
      image = latest_image_;
      info = latest_info_;
      cat_hit = cat_hit_;
      prev = hunter_.phase();
    }

    double d_min = std::numeric_limits<double>::infinity();
    double left_open = 0.0;
    double right_open = 0.0;
    if (scan && !scan->ranges.empty()) {
      ScanView view;
      view.ranges = scan->ranges.data();
      view.n = scan->ranges.size();
      view.angle_min = scan->angle_min;
      view.angle_increment = scan->angle_increment;
      view.range_min = scan->range_min;
      view.range_max = scan->range_max;
      const auto sector = lidar_front(view, hunter_.config());
      if (sector.have_hit) {
        d_min = sector.d_min;
      }
      left_open = sector.left_median;
      right_open = sector.right_median;
    }

    if (cat_hit && !cat_hit->pose_in_base) {
      fill_cat_base_pose(*cat_hit, info);
    }
    std::vector<std::optional<TargetHit>> hits;
    hits.push_back(saveli_from_tf());
    hits.push_back(cat_hit);
    std::string sticky;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sticky = hunter_.sticky_id();
    }
    const auto hit = pick_target(hits, sticky, enabled_targets_);

    const double t = now().seconds();
    bool pan_done = pan_sweep_done_;
    HunterOutput out;
    bool play = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (player_.finished() && hunter_.phase() == HunterPhase::Name) {
        hunter_.notify_name_done();
      }
      out = hunter_.tick(t, hit, d_min, pan_done, left_open, right_open);
      play = out.play_name;
      if (out.phase == HunterPhase::Hunt && prev != HunterPhase::Hunt) {
        hunt_pan_t0_ = t;
        pan_sweep_done_ = false;
      }
      last_out_ = out;
      if (out.legs == LegCommandKind::Twist) {
        walking_active_ = true;
        last_walk_cmd_at_ = now();
      } else if (out.legs == LegCommandKind::Halt) {
        walking_active_ = false;
      }
    }

    if (play) {
      std::string wav = hit ? hit->spoken_wav : saveli_wav_;
      player_.play_once(wav, 3.0);
    }

    apply_legs(out);
    apply_head(out, hit, info);
    publish_overlay(out, hit, d_min, image);
  }

  void apply_legs(const HunterOutput &out) {
    if (out.legs == LegCommandKind::Halt) {
      // Idle/Hunt halt every 50 ms would spam gait=0/-2. Once is a stand.
      if (last_applied_legs_ != LegCommandKind::Halt) {
        halt_legs();
      }
      last_applied_legs_ = LegCommandKind::Halt;
      return;
    }
    last_applied_legs_ = out.legs;
    if (out.legs == LegCommandKind::SelectGait15) {
      select_gait15();
      return;
    }
    if (out.legs == LegCommandKind::Twist) {
      if (!hunter_.config().enable_walk) {
        return;
      }
      geometry_msgs::msg::Twist tw;
      tw.linear.x = out.twist.vx;
      tw.linear.y = out.twist.vy;
      tw.angular.z = out.twist.wz;
      if (dry_run_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "dry_run twist vx=%.3f vy=%.3f wz=%.3f (not sent)", tw.linear.x,
                             tw.linear.y, tw.angular.z);
        return;
      }
      cmd_vel_pub_->publish(tw);
    }
  }

  void apply_head(const HunterOutput &out, const std::optional<TargetHit> &hit,
                  sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
    ArmPulses arm = rest_;
    arm.duration_s = 0.08;
    if (out.pan_head) {
      const double t = now().seconds();
      const double elapsed = t - hunt_pan_t0_;
      const double period = pan_period_s_ > 1.0 ? pan_period_s_ : 10.0;
      const double phase = std::fmod(std::max(0.0, elapsed) / period, 1.0);
      float id19 = rest_.id19;
      if (phase < 0.5) {
        id19 = pan_min_ + (pan_max_ - pan_min_) * static_cast<float>(phase * 2.0);
      } else {
        id19 = pan_max_ - (pan_max_ - pan_min_) * static_cast<float>((phase - 0.5) * 2.0);
      }
      arm.id19 = id19;
      gaze_id19_ = id19;
      gaze_id22_ = rest_.id22;
      arm.id22 = rest_.id22;
      if (elapsed >= period) {
        pan_sweep_done_ = true;
      }
    } else if (out.phase == HunterPhase::Follow && hit && hit->pose.has_pixel && info &&
               info->k.size() >= 9) {
      CameraIntrinsics K = from_k_matrix(static_cast<int>(info->width),
                                         static_cast<int>(info->height), info->k.data());
      const Eigen::Vector3d ray = pixel_to_ray(hit->pose.u, hit->pose.v, K);
      const GazeAngles err = ray_to_yaw_pitch(ray);
      const GazePulses next =
          integrate_gaze({gaze_id19_, gaze_id22_}, err, pulse_map_, gaze_gain_, max_step_pulses_,
                         deadband_rad_);
      gaze_id19_ = next.id19;
      gaze_id22_ = next.id22;
      arm.id19 = next.id19;
      arm.id22 = next.id22;
    } else {
      arm.id19 = gaze_id19_;
      arm.id22 = gaze_id22_;
    }
    if (dry_run_) {
      return;
    }
    if (out.phase == HunterPhase::Idle) {
      arm = rest_;
      arm.duration_s = rest_.duration_s;
      gaze_id19_ = rest_.id19;
      gaze_id22_ = rest_.id22;
    }
    servo_pub_->publish(make_arm_command(arm));
  }

  void publish_overlay(const HunterOutput &out, const std::optional<TargetHit> &hit, double d_min,
                       sensor_msgs::msg::Image::ConstSharedPtr image) {
    if (!image || image->data.empty() || image->width == 0 || image->height == 0) {
      return;
    }
    cv_bridge::CvImagePtr cv;
    try {
      cv = cv_bridge::toCvCopy(image, "bgr8");
    } catch (const cv_bridge::Exception &) {
      return;
    } catch (const std::exception &) {
      return;
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s r=%.2f th=%.1f dmin=%.2f left=%.0f %s",
                  hunter_phase_name(out.phase), out.r, out.theta * 180.0 / kHunterPi, d_min,
                  out.follow_left_s, out.wander_action);
    cv::putText(cv->image, buf, {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    if (hit && hit->pose.has_pixel) {
      cv::circle(cv->image, {static_cast<int>(hit->pose.u), static_cast<int>(hit->pose.v)}, 8,
                 {0, 0, 255}, 2);
      cv::putText(cv->image, hit->id, {static_cast<int>(hit->pose.u), static_cast<int>(hit->pose.v) - 12},
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2);
    }
    image_pub_->publish(*cv->toImageMsg());
  }

  void select_gait15() {
    if (dry_run_) {
      RCLCPP_INFO(get_logger(), "dry_run Traveling gait=%d (not sent)", follow_gait_select_);
      return;
    }
    kinematics_msgs::msg::Traveling sel;
    sel.gait = static_cast<int8_t>(follow_gait_select_);
    sel.height = cmd_height_;
    sel.time = cmd_period_;
    sel.stride = 0.0f;
    sel.interrupt = true;
    traveling_pub_->publish(sel);
  }

  void halt_legs() {
    walking_active_ = false;
    last_applied_legs_ = LegCommandKind::Halt;
    if (dry_run_) {
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

  void watchdog_tick() {
    bool should_halt = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || !walking_active_ || walk_watchdog_s_ <= 0.0) {
        return;
      }
      if (last_walk_cmd_at_.nanoseconds() == 0) {
        return;
      }
      if ((now() - last_walk_cmd_at_).seconds() <= walk_watchdog_s_) {
        return;
      }
      walking_active_ = false;
      should_halt = true;
    }
    if (should_halt) {
      halt_legs();
      RCLCPP_WARN(get_logger(),
                  "walk watchdog: no cmd_vel for %.2f s; halt_legs (Traveling, not zero Twist)",
                  walk_watchdog_s_);
    }
  }

  void on_start(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    halt_legs();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = false;
      hunter_.start();
      hunt_pan_t0_ = now().seconds();
      pan_sweep_done_ = false;
      gaze_id19_ = rest_.id19;
      gaze_id22_ = rest_.id22;
    }
    send_rest();
    response->success = true;
    response->message = "hunt";
    RCLCPP_INFO(get_logger(), "start: HUNT (head sweep). enable_walk=%s",
                hunter_.config().enable_walk ? "true" : "false");
  }

  void on_stop(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      hunter_.stop();
    }
    player_.stop();
    halt_legs();
    send_rest();
    response->success = true;
    response->message = "idle";
  }

  void send_rest() {
    if (dry_run_) {
      return;
    }
    servo_pub_->publish(make_arm_command(rest_));
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string scan_topic_;
  std::string apriltag_topic_;
  std::string servo_topic_;
  std::string cmd_vel_topic_;
  std::string traveling_topic_;
  std::string camera_frame_;
  std::string base_frame_;
  std::string saveli_tag_frame_;
  std::string voice_dir_;
  std::string saveli_wav_;
  std::string cat_wav_;
  std::vector<std::string> enabled_targets_;
  int saveli_tag_id_{0};
  int follow_gait_select_{15};
  bool dry_run_{false};
  float pan_min_{200.0f};
  float pan_max_{800.0f};
  float cmd_height_{25.0f};
  float cmd_period_{0.70f};
  double pan_period_s_{10.0};
  double gaze_gain_{0.10};
  double max_step_pulses_{8.0};
  double deadband_rad_{0.06};
  double walk_watchdog_s_{0.3};
  double hunt_pan_t0_{0.0};
  bool pan_sweep_done_{false};
  bool stopping_{false};
  bool walking_active_{false};
  LegCommandKind last_applied_legs_{LegCommandKind::None};
  float gaze_id19_{500.0f};
  float gaze_id22_{150.0f};
  ArmPulses rest_;
  PulseMapping pulse_map_;
  Hunter hunter_;
  HunterOutput last_out_;
  std::unique_ptr<SaveliSource> saveli_;
  std::unique_ptr<CatSource> cat_;
  std::optional<TargetHit> cat_hit_;
  WavPlayer player_;
  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Time last_walk_cmd_at_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::CallbackGroup::SharedPtr detect_cb_group_;
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
  rclcpp::CallbackGroup::SharedPtr watchdog_cb_group_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Publisher<servo_controller_msgs::msg::ServosPosition>::SharedPtr servo_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<kinematics_msgs::msg::Traveling>::SharedPtr traveling_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr detect_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace proud_up

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<proud_up::MashaHunterNode>();
    std::weak_ptr<proud_up::MashaHunterNode> weak = node;
    rclcpp::on_shutdown([weak]() {
      if (auto n = weak.lock()) {
        n->emergency_stop();
      }
    });
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    node->emergency_stop();
    node.reset();
  } catch (const std::exception &e) {
    fprintf(stderr, "masha_hunter_node failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
