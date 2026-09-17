#pragma once

// Hunter policy — no ROS. The node (masha_hunter_node.cpp) calls tick()
// from a timer after it has copied the latest pose / scan. Tests call
// the same functions with fake numbers — no robot required.
//
// Phase machine:
//
//   IDLE --~/start--> HUNT --hit--> NAME --wav done--> FOLLOW
//                         ^                              |
//                         +-- lost / 60 s / STOPPED -----+
//
// Legs. Zero Twist on /controller/cmd_vel *starts a gait in place*.
// Halt is Traveling gait=0 then gait=-2 (stand). This library never
// returns Twist{0,0,0} as a stop; it returns LegCommandKind::Halt.

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace proud_up {

inline constexpr double kHunterPi = 3.14159265358979323846;

enum class HunterPhase { Idle, Hunt, Name, Follow, Stopped };

enum class LegCommandKind {
  None,          // do not publish walk
  Halt,          // Traveling 0 then -2. Never Twist{}
  SelectGait15,  // Traveling gait=15 once so cmd_gait=5
  Twist          // publish vx, vy, wz on /controller/cmd_vel
};

struct PoseInBase {
  double x{0.0};  // m, +X head
  double y{0.0};  // m, +Y left
  double yaw{0.0};
  double u{0.0};
  double v{0.0};
  bool has_pixel{false};
  bool range_ok{true};
};

struct TargetHit {
  std::string id;            // "saveli" | "cat"
  std::string display_name;
  std::string spoken_wav;    // basename or absolute path
  PoseInBase pose;
  double range_m{0.0};       // along camera ray if pose is not yet in base_link
  bool pose_in_base{false};
  double confidence{1.0};
};

struct HunterConfig {
  double r_target{0.80};
  double d_stop{0.55};
  double d_go{0.70};
  double lost_timeout{1.5};  // Follow→Hunt; brief TF drops coast until then
  double follow_max_s{60.0};
  double search_timeout_s{20.0};
  double name_timeout_s{2.5};
  double vx_max{0.05};
  double vy_max{0.04};
  double wz_max{0.30};
  double kp_x{0.35};
  double kd_x{0.05};
  double kp_y{0.50};
  double kd_y{0.05};
  double kp_th{1.20};
  double kd_th{0.10};
  double deadband_x{0.05};
  double deadband_y{0.03};
  double deadband_th{5.0 * kHunterPi / 180.0};
  double theta_crab{15.0 * kHunterPi / 180.0};
  double y_on{0.06};
  double y_off{0.03};
  bool enable_walk{false};
  bool enable_crab{false};
  bool enable_wander{false};
  double wander_vx{0.03};
  double wander_wz{0.25};
  double control_dt{0.05};
  double lidar_x{0.102};
  double lidar_y{0.0};
  double lidar_ignore_below{0.20};
  double lidar_sector_rad{90.0 * kHunterPi / 180.0};
};

struct TwistBody {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

inline bool twist_is_zero(const TwistBody &t) {
  return t.vx == 0.0 && t.vy == 0.0 && t.wz == 0.0;
}

struct FollowErrors {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double r{0.0};
};

struct LidarSector {
  double d_min{0.0};  // +inf if no finite hit
  double left_median{0.0};
  double right_median{0.0};
  bool have_hit{false};
};

struct ScanView {
  const float *ranges{nullptr};
  std::size_t n{0};
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

struct HunterOutput {
  HunterPhase phase{HunterPhase::Idle};
  LegCommandKind legs{LegCommandKind::None};
  TwistBody twist;
  bool pan_head{false};
  bool play_name{false};
  bool crab{false};
  std::string sticky_id;
  double r{0.0};
  double theta{0.0};
  double d_min{0.0};
  double follow_left_s{0.0};
  const char *wander_action{"hold"};
};

const char *hunter_phase_name(HunterPhase p);

FollowErrors errors_from_pose(const PoseInBase &p, double r_target);

double clamp_val(double v, double lo, double hi);

double pd_axis(double e, double e_dot, double kp, double kd, double deadband, double limit);

LidarSector lidar_front(const ScanView &scan, const HunterConfig &cfg);

std::optional<TargetHit> pick_target(const std::vector<std::optional<TargetHit>> &hits,
                                     const std::string &sticky_id,
                                     const std::vector<std::string> &order);

class Hunter {
 public:
  explicit Hunter(HunterConfig cfg = {});

  void start();
  void stop();
  void notify_name_done();
  // Walk watchdog (or any external halt) stood the legs. Next Twist
  // must SelectGait15 again.
  void notify_halted();
  void set_config(const HunterConfig &cfg) { cfg_ = cfg; }
  const HunterConfig &config() const { return cfg_; }

  HunterOutput tick(double now_s, const std::optional<TargetHit> &hit, double d_min,
                    bool pan_sweep_done = true, double left_open = 0.0,
                    double right_open = 0.0);

  HunterPhase phase() const { return phase_; }
  std::string sticky_id() const { return sticky_id_; }

 private:
  void enter(HunterPhase p, double now_s);
  TwistBody follow_twist(const PoseInBase &pose, double dt);
  HunterOutput hunt_motion(double d_min, bool pan_sweep_done, double left_open,
                           double right_open);

  HunterConfig cfg_;
  HunterPhase phase_{HunterPhase::Idle};
  bool crab_{false};
  bool gait15_sent_{false};
  bool named_this_lock_{false};
  bool name_playing_{false};
  bool lidar_latched_{false};
  bool ignore_until_sweep_{false};
  bool wall_stood_{false};
  int wander_kind_{0};  // 0 hold, 1 walk, 2 turn
  double now_{0.0};
  double phase_t_{0.0};
  double follow_t0_{0.0};
  double last_hit_t_{0.0};
  double last_named_t_{-1.0e9};
  double prev_ex_{0.0};
  double prev_ey_{0.0};
  double prev_eth_{0.0};
  bool have_prev_e_{false};
  std::string sticky_id_;
  std::string last_named_id_;
  std::string pending_wav_;
  TwistBody last_twist_{};
};

}  // namespace proud_up
