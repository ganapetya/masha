// masha_hunter_node — spider hunter on Masha (Jetson Orin NX, ROS 2 Humble).
//
// Two-layer design (this is the C++ lesson the tests rely on):
//
//   hunter.hpp / hunter.cpp   — the BRAIN. No rclcpp. tick() in, Halt/Twist out.
//   audio_player.hpp          — the SPEAKER. Worker thread, aplay, ffplay.
//   THIS FILE                 — the BODY. Topics, TF, servos, overlay.
//
// gtest calls Hunter::tick() with fake numbers. The policy does not need
// a robot, a camera, or this node to be proven. This file only *wires*
// live sensors into that same tick().
//
// Phase machine (owned by Hunter, executed here):
//
//   IDLE --~/start--> HUNT --hit--> NAME --mp3 done--> FOLLOW
//                         ^                              |
//                         +-- lost (lost_timeout) -------+
//
// Follow keeps the lock until it is lost. The head does not pan away
// on a timer to look for another target. LiDAR Stopped stands in place.
//
// ---------------------------------------------------------------------------
// Order of operations — process lifetime
// ---------------------------------------------------------------------------
//
//  1. `ros2 launch proud_up masha_hunter.launch.py` starts this executable.
//     If enabled_targets contains "saveli", the launch file ALSO starts
//     apriltag_ros as a composable node (AprilTagNode). That other node
//     publishes TF child `saveli_tag`. This node never runs the detector.
//  2. main() → rclcpp::init → construct MashaHunterNode → executor.spin().
//     spin() is the ROS 2 event loop. It never returns until Ctrl-C.
//  3. The constructor DECLARES parameters and CREATES pubs/subs/services/
//     timers. It does NOT start hunting. Phase stays Idle until ~/start.
//     (declare_parameter is how ROS 2 binds yaml + launch args to C++.)
//  4. Operator:  ros2 service call /masha_hunter_node/start std_srvs/srv/Trigger
//     → on_start() → hunter_.start() → phase Hunt, head begins to sweep.
//  5. Every 50 ms, control_tick() copies the latest sensors, calls
//     Hunter::tick(), then publishes legs / head / overlay.
//  6. ~/stop or Ctrl-C → emergency_stop(): halt legs with Traveling
//     (gait 0 then -2), kill audio, freeze policy in Idle.
//     Zero Twist on /controller/cmd_vel is NOT a halt — it starts a gait
//     in place. That is why we never publish Twist{0,0,0} as a stop.
//
// ---------------------------------------------------------------------------
// Order of operations — one control_tick (20 Hz)
// ---------------------------------------------------------------------------
//
//  A. Short mutex: copy latest scan / image / camera_info / cat_hit pointers.
//     Shared_ptr copy is cheap (refcount). We do NOT copy image bytes here.
//  B. LiDAR: lidar_front() walks /scan ranges → d_min, left/right openness.
//  C. Cat (optional): if YOLO already found a pixel, fill_cat_base_pose()
//     turns (u,v,depth) into (x,y) in base_link via TF camera→base.
//  D. Saveli: saveli_from_tf() looks up T_base_tag. Stale TF is not a hit.
//  E. pick_target(): preempt_rank, else sticky id, else yaml order.
//     A cat pixel is offered only after kCatConfirmTicks centres in a row.
//     One empty frame keeps that streak. The cat gate covers the whole
//     frame: the head is moving, so the same cat is not a new object.
//  F. If a NAME clip just finished, hunter_.notify_name_done(t).
//     That starts the cat silence clock in Follow as well as in Name.
//  G. hunter_.tick(...) — THE policy. Returns HunterOutput.
//  H. Hunt entered from Follow: start the pan sweep at the *current* pan.
//     Snapping servo 19 to pan_min (200) looks ~70° away and loses the tag.
//  I. play_name → AudioPlayer::play_once. This timer never spawns aplay.
//  J. apply_legs()  → Halt (Traveling) / SelectGait15 / Twist (cmd_vel)
//  K. apply_head()  → Hunt with no cat: triangle pan.
//     Cat pixel: gaze until centred, then hold. A still cat does not scan.
//  L. publish_overlay() → ~/image_result (rqt / foxglove).
//
// ---------------------------------------------------------------------------
// Threads — why MultiThreadedExecutor and callback groups
// ---------------------------------------------------------------------------
//
// One ROS 2 executor thread cannot do YOLO (tens of ms) AND keep 20 Hz
// cmd_vel. If OpenCV ran inside on_image(), LaserScan would pile up and
// the walk watchdog would stand the legs.
//
//   image_cb_group_     DDS callbacks: store the latest shared_ptr, return.
//   detect_cb_group_    50 ms timer: YOLO for the cat plug (no-op if cat off).
//   control_cb_group_   50 ms timer: TF + tick + publish.
//   watchdog_cb_group_  100 ms: if cmd_vel went silent, stand the legs.
//   service_cb_group_   ~/start and ~/stop (so start is not blocked by YOLO).
//
// MutuallyExclusive *inside* a group: at most one callback of that group
// runs at a time. Different groups CAN run in parallel — that is why
// mutex_ exists. Image callbacks never call OpenCV, TF, or Hunter.
//
// Do not also launch follow_the_cat_node, masha_interaction FOLLOW,
// lidar_app, Nav2, or analog joystick. Two writers on /controller/cmd_vel
// and on servos 19/22 fight.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
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

#include "proud_up/audio_player.hpp"
#include "proud_up/follow_the_cat.hpp"
#include "proud_up/hunter.hpp"
#include "proud_up/pose_commander.hpp"
#include "proud_up/reach_greet.hpp"
#include "proud_up/target_source.hpp"

using namespace std::chrono_literals;

namespace proud_up {
namespace {

// TF rotation is a quaternion. getRPY is ZYX Euler; yaw is rotation
// about +Z (up) — that is the heading we feed into PoseInBase::yaw.
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
  // Member-initializer list runs BEFORE the constructor body:
  //   Node("masha_hunter_node")  ROS graph name; ~/start → /masha_hunter_node/start
  //   tf_buffer_(get_clock())    uses this node's clock (sim-time ready)
  //   tf_listener_(tf_buffer_)   background subscriber to /tf and /tf_static
  //
  // Constructor body order (do not reorder casually):
  //   1. declare_parameter — yaml + launch args bind here
  //   2. build HunterConfig, construct hunter_
  //   3. arm rest pose + pan limits
  //   4. TargetSource plugs (Saveli always, Cat only if enabled)
  //   5. callback groups, then publishers, then subscriptions, then services
  //   6. timers last — they can fire as soon as the executor spins
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

    // Launch extra dict overrides yaml, yaml overrides these C++ defaults.
    enabled_targets_ = declare_parameter<std::vector<std::string>>("enabled_targets", {"saveli"});
    saveli_tag_id_ = declare_parameter<int>("saveli_tag_id", 0);
    dry_run_ = declare_parameter<bool>("dry_run", false);

    HunterConfig cfg;
    cfg.enable_walk = declare_parameter<bool>("enable_walk", false);
    cfg.enable_crab = declare_parameter<bool>("enable_crab", true);
    cfg.enable_wander = declare_parameter<bool>("enable_wander", false);
    cfg.r_target = declare_parameter<double>("r_target", 0.15);
    cfg.d_stop = declare_parameter<double>("d_stop", 0.55);
    cfg.d_go = declare_parameter<double>("d_go", 0.70);
    cfg.lost_timeout = declare_parameter<double>("lost_timeout", 1.5);
    pose_max_age_s_ = declare_parameter<double>("pose_max_age", 0.40);
    cfg.search_timeout_s = declare_parameter<double>("search_timeout_s", 20.0);
    cfg.name_timeout_s = declare_parameter<double>("name_timeout_s", 8.0);
    cfg.cat_greet_silence_s = declare_parameter<double>("cat_greet_silence_s", 5.0);
    cfg.vx_max = declare_parameter<double>("vx_max", 0.25);
    cfg.vy_max = declare_parameter<double>("vy_max", 0.16);
    cfg.wz_max = declare_parameter<double>("wz_max", 0.30);
    cfg.kp_x = declare_parameter<double>("kp_x", 0.60);
    cfg.kd_x = declare_parameter<double>("kd_x", 0.08);
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

    // Bus pulses are integers 0–1000; yaml is double so launch can override.
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
    cmd_height_ = static_cast<float>(declare_parameter<double>("cmd_height", 35.0));
    cmd_period_ = static_cast<float>(declare_parameter<double>("cmd_period", 0.60));
    gaze_gain_ = declare_parameter<double>("gaze_gain", 0.10);
    max_step_pulses_ = declare_parameter<double>("max_step_pulses", 8.0);
    deadband_rad_ = declare_parameter<double>("deadband_rad", 0.06);
    // YOLO on the white cat gaps for more than 1 s. Dropping the hit at
    // 1 s resumed the head sweep, the cat left the picture, and the next
    // sighting was another drive-by. Hold the last hit long enough to
    // centre and keep walking; search again only after this.
    cat_hold_s_ = declare_parameter<double>("cat_hold_s", 3.0);
    // integrate_gaze on a pixel older than this never sees the error
    // shrink (the image is frozen), so servo 19 runs to the stop.
    cat_fresh_s_ = declare_parameter<double>("cat_fresh_s", 0.40);
    cat_relock_px_ = declare_parameter<double>("cat_relock_px", 80.0);
    pulse_map_.yaw_sign = declare_parameter<double>("yaw_sign", -1.0);
    pulse_map_.pitch_sign = declare_parameter<double>("pitch_sign", 1.0);
    pulse_map_.pan_min = pan_min_;
    pulse_map_.pan_max = pan_max_;
    pulse_map_.pan_rest = rest_.id19;
    pulse_map_.tilt_rest = rest_.id22;
    gaze_id19_ = rest_.id19;
    gaze_id22_ = rest_.id22;

    if (voice_dir_.empty()) {
      // src, not install/share: Peter copies call-*.mp3 into the package tree.
      voice_dir_ = "/home/ubuntu/ros2_ws/src/xf_mic_asr_offline/feedback_voice/english";
    }

    saveli_wav_ =
        declare_parameter<std::string>("saveli_wav", voice_dir_ + "/call-savelij.mp3");
    cat_wav_ = declare_parameter<std::string>("cat_wav", voice_dir_ + "/call-kitten.mp3");
    name_play_timeout_s_ = declare_parameter<double>("name_play_timeout_s", 8.0);

    // unique_ptr: exclusive ownership, destroyed with the node. Saveli
    // is cheap (no DNN). Cat loads yolov8n.onnx — only construct if asked.
    saveli_ = std::make_unique<SaveliSource>(saveli_tag_id_, saveli_wav_);
    saveli_->set_enabled(target_enabled("saveli"));

    SubjectDetectConfig cat_cfg;
    cat_cfg.coco_class = 15;  // COCO: 0=person, 15=cat. Same ONNX file.
    cat_cfg.require_person_shape = false;
    cat_cfg.enable_face_fallback = false;
    cat_cfg.person_head_frac = 0.50;
    cat_cfg.person_imgsz = declare_parameter<int>("person_imgsz", 320);
    cat_cfg.person_conf = declare_parameter<double>("person_conf", 0.25);
    cat_cfg.dnn_cuda = declare_parameter<bool>("dnn_cuda", true);
    // follow-the-cat uses 0.22 so a person lock does not hop to a
    // neighbour. The hunter pans servo 19 and then walks, so the same
    // cat crosses most of the picture between YOLO frames. A tight gate
    // threw that box away and the red circle vanished.
    cat_cfg.track_gate_frac = 1.05;
    cat_track_gate_frac_ = cat_cfg.track_gate_frac;
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
      cat_ = std::make_unique<CatSource>(cat_cfg, cat_wav_, cfg.cat_greet_silence_s);
      cat_->set_enabled(true);
    }

    // MutuallyExclusive = one callback of THIS group at a time.
    // Separate groups so YOLO cannot stall cmd_vel or ~/start.
    image_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    detect_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Queue depth 1: latest command wins. Depth 10 would replay stale Twists.
    servo_pub_ = create_publisher<servo_controller_msgs::msg::ServosPosition>(servo_topic_, 1);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
    traveling_pub_ = create_publisher<kinematics_msgs::msg::Traveling>(traveling_topic_, 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_result", 1);

    // SensorDataQoS = BEST_EFFORT, keep-last 5. Camera/LiDAR must not
    // block the publisher if this node is slow. RELIABLE would stall them.
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

    // Wall timers use steady time, not /clock. 50 ms = 20 Hz = control_dt.
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
    RCLCPP_INFO(get_logger(), "NAME saveli=%s (%s) cat=%s (%s) cat_silence=%.1fs",
                saveli_wav_.c_str(), AudioPlayer::clip_exists(saveli_wav_) ? "ok" : "MISSING",
                cat_wav_.c_str(), AudioPlayer::clip_exists(cat_wav_) ? "ok" : "MISSING",
                hunter_.config().cat_greet_silence_s);
  }

  ~MashaHunterNode() override { emergency_stop(); }

  // Safe to call from destructor, on_shutdown, and exceptions. Halt the
  // policy first (under mutex) so a concurrent control_tick sees stopping_
  // and returns, then kill audio, then stand. Order matters: if we halt
  // legs first, tick could publish Twist again before stopping_ is set.
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

  // DDS callbacks: latest-wins. ConstSharedPtr is a refcounted pointer
  // to the serialized image already in memory — we do not memcpy pixels
  // here. lock_guard is RAII: mutex unlocks if this function returns OR
  // throws. std::move transfers the pointer so the callback argument is
  // empty after the assignment (the previous latest_image_ is released).
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

  // Swallow exceptions so one bad OpenCV frame cannot kill the node
  // (and take ~/start with it). The executor would otherwise unwind.
  void detect_tick() {
    try {
      detect_tick_inner();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "detect_tick: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "detect_tick: unknown exception");
    }
  }

  // Cat plug only. Saveli pose is TF, looked up on the control timer —
  // YOLO must not share that thread. Lock is held only while copying
  // pointers; DNN runs unlocked so control_tick can still take mutex_.
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
    // Pointers into stack cv::Mat. CatSource::detect must finish before
    // these go out of scope — it does; we do not store `in` on the node.
    DetectInput in;
    in.bgr = &cv->image;
    in.depth_mm = depth_mm.empty() ? nullptr : &depth_mm;
    if (info && info->k.size() >= 9) {
      in.K = from_k_matrix(static_cast<int>(info->width), static_cast<int>(info->height),
                           info->k.data());
    }
    auto hit = cat_->detect(in);
    const double diag = std::hypot(static_cast<double>(cv->image.cols),
                                   static_cast<double>(cv->image.rows));
    const double gate = cat_track_gate_frac_ * diag;
    auto confirmed = confirm_cat_hit(cat_confirm_, hit, gate, kCatConfirmTicks);
    const double stamp = now().seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    if (confirmed) {
      cat_hit_ = std::move(confirmed);
      cat_hit_stamp_s_ = stamp;
    }
    // A miss leaves the last confirmed hit. control_tick drops it when
    // the stamp is older than the cat hold, so one empty YOLO frame does
    // not erase the red circle or the follow pose.
  }

  // Pixel (u,v) is a DIRECTION, not a 3D point. Scale the unit ray by
  // measured depth, then TF from camera optical frame into base_link:
  //   p_base = T_base_camera * (ray * range_m)
  // TimePointZero = "latest available transform", not the image stamp.
  // That is slightly wrong under motion, but avoids waiting on TF.
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

  // Saveli hit = a FRESH TF child. We do not subscribe to
  // apriltag_msgs: Humble vs the Jetson overlay ship different
  // AprilTagDetection layouts; the wrong one SIGSEGVs this node.
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
      // lookupTransform returns the last published pose even after the
      // tag left the image. Age > pose_max_age means "coast, do not treat
      // this as a live hit" — otherwise Follow never goes lost.
      const double age = (now() - rclcpp::Time(tf.header.stamp)).seconds();
      const double max_age = pose_max_age_s_ > 0.05 ? pose_max_age_s_ : 0.40;
      if (age > max_age) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Saveli TF stale (%.2f s > %.2f). Duplicate id-0 or tag left the image.",
            age, max_age);
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

  // See the file header for A–L. This function is the 20 Hz heartbeat.
  void control_tick_inner() {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    sensor_msgs::msg::Image::ConstSharedPtr image;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    std::optional<TargetHit> cat_hit;
    double cat_stamp_s = 0.0;
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
      cat_stamp_s = cat_hit_stamp_s_;
      prev = hunter_.phase();
    }
    bool cat_fresh = false;
    if (cat_hit) {
      // Longer than Saveli's pose_max_age, and longer than one missed
      // YOLO frame. The head must stay on the last cat while this lasts
      // so a sitting cat is not scanned out of the picture.
      const double max_age = cat_hold_s_ > 0.2 ? cat_hold_s_ : 3.0;
      const double age = now().seconds() - cat_stamp_s;
      cat_fresh = cat_hit->policy.gaze_on_pixel && age <= cat_fresh_s_;
      if (age > max_age) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "cat hit stale (%.2f s > %.2f). Search again.", age, max_age);
        cat_hit.reset();
        cat_fresh = false;
        cat_head_latched_ = false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (cat_hit_stamp_s_ == cat_stamp_s) {
          cat_hit_.reset();
        }
      }
    }

    double d_min = std::numeric_limits<double>::infinity();
    double left_open = 0.0;
    double right_open = 0.0;
    // Kept so the 1 Hz line can name the bearing of the closest return.
    // The stop itself still uses only d_min, inside Hunter::tick.
    LidarSector sector;
    sector.d_min = d_min;
    if (scan && !scan->ranges.empty()) {
      ScanView view;
      view.ranges = scan->ranges.data();
      view.n = scan->ranges.size();
      view.angle_min = scan->angle_min;
      view.angle_increment = scan->angle_increment;
      view.range_min = scan->range_min;
      view.range_max = scan->range_max;
      sector = lidar_front(view, hunter_.config());
      if (sector.have_hit) {
        d_min = sector.d_min;
      }
      left_open = sector.left_median;
      right_open = sector.right_median;
    }

    if (cat_hit && !cat_hit->pose_in_base) {
      fill_cat_base_pose(*cat_hit, info);
    }
    // One slot per plug. This order is not priority. pick_target uses
    // TargetPolicy::preempt_rank, then the sticky lock, then enabled_targets.
    // A dog or a second tag is another optional<TargetHit> here, with that
    // plug's policy already stamped. Hunter::tick does not learn the name.
    std::vector<std::optional<TargetHit>> hits;
    hits.push_back(saveli_from_tf());
    hits.push_back(cat_hit);
    std::string sticky;
    TargetPolicy sticky_policy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sticky = hunter_.sticky_id();
      sticky_policy = hunter_.sticky_policy();
    }
    const auto hit = pick_target(hits, sticky, enabled_targets_, sticky_policy);

    const double t = now().seconds();
    bool pan_done = pan_sweep_done_;
    HunterOutput out;
    bool play = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (player_.finished() && hunter_.name_playing()) {
        hunter_.notify_name_done(t);
      }
      // Policy. hit / d_min / pan_done are already in library types.
      out = hunter_.tick(t, hit, d_min, pan_done, left_open, right_open);
      // Pixel-only cat (or bbox-height guess) may NAME and gaze. It must
      // not SelectGait15 / Twist toward a pose that is still (0, 0).
      out.legs = legs_for_metric_target(out.legs, hit);
      play = out.play_name;
      // One line when the legs stand for the lidar, and one line a second
      // while she is hunting. The front wedge is ±45° from the body, so a
      // wall beside her can be the closest return. pan=frozen is the tag
      // lock (or this stop): Hunt is the only phase that sweeps the head.
      if (out.phase == HunterPhase::Stopped && prev != HunterPhase::Stopped) {
        const double bearing_deg =
            sector.have_hit ? sector.d_min_bearing * 180.0 / kHunterPi : 0.0;
        RCLCPP_WARN(get_logger(),
                    "LIDAR stop. closest %.2f m at %+.0f deg "
                    "(0 forward, + left, wedge ±45, d_stop %.2f). "
                    "tag r=%.2f th=%+.0f. Head stays.",
                    sector.have_hit ? sector.d_min : -1.0, bearing_deg,
                    hunter_.config().d_stop, out.r, out.theta * 180.0 / kHunterPi);
      }
      if (out.phase != HunterPhase::Idle) {
        const char *why = "walk";
        if (out.phase == HunterPhase::Stopped) {
          why = "lidar";
        } else if (out.at_standoff) {
          why = "standoff";
        } else if (out.phase == HunterPhase::Name) {
          why = "name";
        } else if (out.phase == HunterPhase::Hunt) {
          why = "search";
        } else if (out.legs == LegCommandKind::Halt) {
          why = "halt";
        }
        const double bearing_deg =
            sector.have_hit ? sector.d_min_bearing * 180.0 / kHunterPi : 0.0;
        const char *legs = "none";
        if (out.legs == LegCommandKind::Halt) {
          legs = "halt";
        } else if (out.legs == LegCommandKind::Twist) {
          legs = "twist";
        } else if (out.legs == LegCommandKind::SelectGait15) {
          legs = "gait15";
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "phase=%s why=%s pan=%s legs=%s d_min=%.2f at %+.0f deg "
            "tag r=%.2f th=%+.0f",
            hunter_phase_name(out.phase), why, out.pan_head ? "sweep" : "frozen", legs,
            sector.have_hit ? sector.d_min : -1.0, bearing_deg, out.r,
            out.theta * 180.0 / kHunterPi);
      }
      if (out.phase == HunterPhase::Hunt && prev != HunterPhase::Hunt) {
        pan_sweep_done_ = false;
        const double period = pan_period_s_ > 1.0 ? pan_period_s_ : 10.0;
        // From Follow, start the sweep at the current pan. Snapping to
        // pan_min (200) looks ~70° away and loses a tag that is still nearby.
        if (prev == HunterPhase::Follow || prev == HunterPhase::Stopped) {
          const float span = pan_max_ - pan_min_;
          float u = 0.5f;
          if (span > 1.0f) {
            u = (gaze_id19_ - pan_min_) / span;
            u = std::max(0.0f, std::min(1.0f, u));
          }
          hunt_pan_t0_ = t - static_cast<double>(0.5f * u) * period;
          RCLCPP_WARN(get_logger(),
                      "FOLLOW→HUNT lost (no fresh tag). Search from pan %.0f, not snap to min.",
                      gaze_id19_);
        } else {
          hunt_pan_t0_ = t;
        }
      }
      last_out_ = out;
      if (out.legs == LegCommandKind::Twist) {
        walking_active_ = true;
        last_walk_cmd_at_ = now();
      } else if (out.legs == LegCommandKind::Halt || out.legs == LegCommandKind::None) {
        walking_active_ = false;
      }
    }

    if (play) {
      std::string wav = hit ? hit->spoken_wav : saveli_wav_;
      RCLCPP_INFO(get_logger(), "NAME play %s", wav.c_str());
      player_.play_once(wav, name_play_timeout_s_);
    }

    apply_legs(out);
    if (!apply_reach_greet(t, out)) {
      apply_head(out, hit, info, cat_fresh);
    }
    publish_overlay(out, hit, d_min, image);
  }

  // Map LegCommandKind → ROS messages. One kind per tick:
  //   Halt         → Traveling 0 then -2, once (not every 50 ms)
  //   SelectGait15 → Traveling gait=15 so the controller's cmd_gait = 5
  //   Twist        → /controller/cmd_vel  (skipped if enable_walk=false)
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
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "%stwist vx=%.3f vy=%.3f wz=%.3f crab=%s r=%.2f th=%.1f",
                           dry_run_ ? "dry_run " : "", tw.linear.x, tw.linear.y, tw.angular.z,
                           out.crab ? "true" : "false", out.r,
                           out.theta * 180.0 / kHunterPi);
      if (dry_run_) {
        return;
      }
      cmd_vel_pub_->publish(tw);
    }
  }

  // True while the reach greeting owns the arm. Hunt pan and gaze stay off
  // for that stretch, including the hold after the head is back at rest.
  bool apply_reach_greet(double now_s, const HunterOutput &out) {
    ReachArm home;
    home.id19 = rest_.id19;
    home.id20 = rest_.id20;
    home.id21 = rest_.id21;
    home.id22 = rest_.id22;
    home.id23 = rest_.id23;
    home.id24 = rest_.id24;
    float pan = rest_.id19;
    pan += static_cast<float>(pulse_map_.yaw_sign * out.theta * pulse_map_.ticks_per_rad);
    pan = std::max(pan_min_, std::min(pan_max_, pan));
    const ReachPhase prev = reach_greet_.phase;
    const ReachGreetOutput greet =
        advance_reach_greet(reach_greet_, out.at_standoff, now_s, home, pan);
    if (greet.publish) {
      ArmPulses arm = rest_;
      arm.id19 = greet.arm.id19;
      arm.id20 = greet.arm.id20;
      arm.id21 = greet.arm.id21;
      arm.id22 = greet.arm.id22;
      arm.id23 = greet.arm.id23;
      arm.id24 = greet.arm.id24;
      arm.duration_s = greet.arm.duration_s;
      if (prev == ReachPhase::Idle && reach_greet_.phase == ReachPhase::Reach) {
        RCLCPP_INFO(get_logger(),
                    "standoff greet: reach, grab twice, home. pan %.0f shoulder %.0f",
                    arm.id19, arm.id20);
      }
      if (dry_run_) {
        RCLCPP_INFO(get_logger(),
                    "dry_run greet id19=%.0f id20=%.0f id22=%.0f id24=%.0f dur=%.2f",
                    arm.id19, arm.id20, arm.id22, arm.id24, arm.duration_s);
      } else {
        servo_pub_->publish(make_arm_command(arm));
      }
      gaze_id19_ = arm.id19;
      gaze_id22_ = arm.id22;
    }
    return greet.own_arm;
  }

  // Head modes, in this order:
  //   Cat pixel   stop the triangle. Gaze until the box is centred, then
  //               hold. A still cat does not rotate the head. A cat that
  //               steps across the picture (cat_relock_px) is aimed again.
  //   Hunt pan    triangle sweep on servo 19, tilt held at rest. Only when
  //               there is no cat pixel — a visible cat must not be scanned.
  //   else        hold the last pan/tilt (Saveli TF, coast with no pixel)
  // duration_s = 0.08 ≈ one control period, so the bus tracks a sweep
  // instead of blending a 1 s move toward a pan that has already changed.
  // cat_fresh is false when the pixel is older than cat_fresh_s_. Do not
  // integrate that frozen error: the head would run to the pan stop.
  void apply_head(const HunterOutput &out, const std::optional<TargetHit> &hit,
                  sensor_msgs::msg::CameraInfo::ConstSharedPtr info, bool cat_fresh) {
    ArmPulses arm = rest_;
    arm.duration_s = 0.08;
    const bool cat_seen = hit && hit->policy.gaze_on_pixel && hit->pose.has_pixel &&
                          out.phase != HunterPhase::Idle;
    const bool can_gaze = cat_seen && info && info->k.size() >= 9;
    if (cat_seen && !can_gaze) {
      // Cat is in the picture but this tick has no camera matrix. Hold.
      // Do not start the hunt triangle.
      arm.id19 = gaze_id19_;
      arm.id22 = gaze_id22_;
    } else if (can_gaze) {
      CameraIntrinsics K = from_k_matrix(static_cast<int>(info->width),
                                         static_cast<int>(info->height), info->k.data());
      const Eigen::Vector3d ray = pixel_to_ray(hit->pose.u, hit->pose.v, K);
      const GazeAngles err = ray_to_yaw_pitch(ray);
      const double shift = cat_head_latched_
                               ? std::hypot(hit->pose.u - cat_head_u_, hit->pose.v - cat_head_v_)
                               : 0.0;
      HeadFocus focus;
      if (cat_fresh) {
        focus = decide_head_focus(cat_head_latched_, err.yaw, err.pitch, deadband_rad_, shift,
                                  cat_relock_px_);
      } else {
        // Remembered cat, no new frame. Hold the aim. Do not scan.
        focus.gaze = false;
        focus.latched = cat_head_latched_;
      }
      if (focus.latched && !cat_head_latched_) {
        cat_head_u_ = hit->pose.u;
        cat_head_v_ = hit->pose.v;
        RCLCPP_INFO(get_logger(),
                    "%s focused: head hold pan %.0f tilt %.0f. Still target, not scanning.",
                    hit->id.c_str(), gaze_id19_, gaze_id22_);
      } else if (!focus.latched && cat_head_latched_) {
        RCLCPP_INFO(get_logger(), "%s moved in the image (%.0f px): re-aim, then hold.",
                    hit->id.c_str(), shift);
      }
      cat_head_latched_ = focus.latched;
      if (focus.gaze) {
        const GazePulses next =
            integrate_gaze({gaze_id19_, gaze_id22_}, err, pulse_map_, gaze_gain_, max_step_pulses_,
                           deadband_rad_);
        gaze_id19_ = next.id19;
        gaze_id22_ = next.id22;
      }
      arm.id19 = gaze_id19_;
      arm.id22 = gaze_id22_;
    } else if (out.pan_head) {
      cat_head_latched_ = false;
      const double t = now().seconds();
      const double elapsed = t - hunt_pan_t0_;
      const double period = pan_period_s_ > 1.0 ? pan_period_s_ : 10.0;
      // Triangle: 0→0.5 goes pan_min→pan_max, 0.5→1.0 comes back.
      // fmod keeps sweeping if Hunt lasts more than one period.
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

  // Debug image on ~/image_result. toCvCopy makes our own BGR so we
  // can draw without mutating the camera's shared buffer.
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
    std::snprintf(buf, sizeof(buf),
                  "%s r=%.2f th=%.1f crab=%d vx=%.2f vy=%.2f dmin=%.2f %s",
                  hunter_phase_name(out.phase), out.r, out.theta * 180.0 / kHunterPi,
                  out.crab ? 1 : 0, out.twist.vx, out.twist.vy, d_min, out.wander_action);
    cv::putText(cv->image, buf, {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    if (hit && hit->pose.has_pixel) {
      cv::circle(cv->image, {static_cast<int>(hit->pose.u), static_cast<int>(hit->pose.v)}, 8,
                 {0, 0, 255}, 2);
      cv::putText(cv->image, hit->id, {static_cast<int>(hit->pose.u), static_cast<int>(hit->pose.v) - 12},
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2);
    }
    image_pub_->publish(*cv->toImageMsg());
  }

  // gait=15 is the *select* code. The controller then runs cmd_gait=5
  // (omnidirectional tripod — the map in follow_gait.cpp). Must be sent
  // once per walk bout, before the first Twist, or cmd_vel is ignored.
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

  // Vendor halt: gait 0 stops the generator, gait -2 is the stand pose.
  // interrupt=true aborts the current half-cycle instead of finishing it.
  // Publishing both back-to-back is how Masha's other apps stand too.
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

  // If Follow is supposed to be walking but cmd_vel went silent (node
  // stall, exception, enable_walk flipped), stand. Zero Twist would
  // march in place — that is the bug this exists to prevent.
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
      hunter_.notify_halted();
      should_halt = true;
    }
    if (should_halt) {
      halt_legs();
      RCLCPP_WARN(get_logger(),
                  "walk watchdog: no cmd_vel for %.2f s; halt_legs (Traveling, not zero Twist)",
                  walk_watchdog_s_);
    }
  }

  // Trigger has empty Request. We only fill Response.{success, message}.
  // Halt first so a leftover Follow Twist cannot keep walking into Hunt.
  void on_start(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    halt_legs();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = false;
      hunter_.start();
      hunt_pan_t0_ = now().seconds();
      pan_sweep_done_ = false;
      cat_head_latched_ = false;
      gaze_id19_ = rest_.id19;
      gaze_id22_ = rest_.id22;
      reach_greet_ = {};
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
      reach_greet_ = {};
    }
    // player_.stop() joins the player thread — do it outside the mutex
    // so control_tick is not blocked for the length of the clip.
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

  // --- parameters (copied out of yaml / launch; not live-reloaded) ---
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
  double name_play_timeout_s_{8.0};
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
  double cat_hold_s_{3.0};           // keep the last cat this long before scanning again
  double cat_fresh_s_{0.40};         // only this new a pixel may move the head
  double cat_relock_px_{80.0};       // latched head re-aims only after a real step
  bool cat_head_latched_{false};     // focused on a still cat; servos 19/22 hold
  double cat_head_u_{0.0};           // pixel where the latch closed
  double cat_head_v_{0.0};
  double walk_watchdog_s_{0.3};
  double pose_max_age_s_{0.40};
  double hunt_pan_t0_{0.0};          // ROS time when the current Hunt sweep started
  bool pan_sweep_done_{false};       // one full triangle completed (wander gate)
  bool stopping_{false};             // set by ~/stop / emergency; ticks no-op
  bool walking_active_{false};       // watchdog only runs while this is true
  LegCommandKind last_applied_legs_{LegCommandKind::None};
  float gaze_id19_{500.0f};          // last commanded pan (integrator state)
  float gaze_id22_{150.0f};          // last commanded tilt
  ArmPulses rest_;                   // hunter pose: arm out, camera forward-down
  ReachGreet reach_greet_{};         // neck reach + two grabs at the 15 cm stop
  PulseMapping pulse_map_;           // optical yaw/pitch → servo ticks
  Hunter hunter_;                    // the ROS-free policy object
  HunterOutput last_out_;
  std::unique_ptr<SaveliSource> saveli_;
  std::unique_ptr<CatSource> cat_;
  std::optional<TargetHit> cat_hit_;  // written by detect_tick, read by control
  double cat_hit_stamp_s_{0.0};       // ROS seconds of the last confirmed cat hit
  CatConfirmState cat_confirm_{};     // detect thread only (detect_cb_group_)
  double cat_track_gate_frac_{0.22};
  AudioPlayer player_;
  std::mutex mutex_;                  // guards latest_* + hunter_ + cat_hit_
  tf2_ros::Buffer tf_buffer_;         // declared before listener (init order)
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Time last_walk_cmd_at_{0, 0, RCL_ROS_TIME};
  // Latest-wins slots. Callbacks overwrite; timers copy the pointer out.
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
  // Parses ROS args (--ros-args -p enable_walk:=true …) then our argc.
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<proud_up::MashaHunterNode>();
    // weak_ptr so the shutdown hook cannot keep the node alive after
    // node.reset() — that would be a shared_ptr cycle.
    std::weak_ptr<proud_up::MashaHunterNode> weak = node;
    rclcpp::on_shutdown([weak]() {
      if (auto n = weak.lock()) {
        n->emergency_stop();
      }
    });
    // MultiThreaded: callback groups run in parallel. SingleThreaded
    // would serialize YOLO and cmd_vel on one thread.
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();  // blocks until Ctrl-C / rclcpp::shutdown
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
