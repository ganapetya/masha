#pragma once

// Hunter policy — no ROS. The node (masha_hunter_node.cpp) calls tick()
// from a 50 ms timer after it has copied the latest pose / scan. Tests
// (test_hunter.cpp) call the same functions with fake numbers — no
// robot, no rclcpp, no camera.
//
// Why a ROS-free library: C++ units of work stay testable. If the PD
// law, the crab gate, or a phase transition is wrong, gtest fails in
// milliseconds. Wiring bugs (wrong topic, stale TF) live in the node.
//
// Phase machine (tick() is the only place these change):
//
//   IDLE --start()--> HUNT --hit--> NAME --notify_name_done()--> FOLLOW
//                         ^                                       |
//                         +-- lost / 60 s / LiDAR STOPPED --------+
//
// tick() inputs (what the node must gather BEFORE calling):
//   now_s          ROS clock seconds (monotonic enough for dt)
//   hit            Saveli and/or cat, already in base_link, or nullopt
//   d_min          closest LiDAR hit in the front 90° sector, metres
//   pan_sweep_done true after one full Hunt pan triangle (node tracks this)
//   left_open / right_open  median LiDAR range each side (wander turn)
//
// tick() output: phase, how to move the legs, whether to pan the head,
// whether to play the NAME clip. The node *applies* that output; this
// file never publishes.
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

// Idle     constructed, or after ~/stop. No motion, no pan.
// Hunt     looking: head triangle-pan. Optional wander walk.
// Name     freeze, play call-savelij / call-kitten, then Follow.
// Follow   PD on range + bearing for ≤ follow_max_s.
// Stopped  LiDAR closer than d_stop; stand until d_go or lost.
enum class HunterPhase { Idle, Hunt, Name, Follow, Stopped };

// What the node should publish this tick. One kind only — never Halt AND Twist.
enum class LegCommandKind {
  None,          // enable_walk=false: log Twist, do not publish
  Halt,          // Traveling gait=0 then gait=-2. Never Twist{}
  SelectGait15,  // Traveling gait=15 once so the controller's cmd_gait becomes 5
  Twist          // publish vx, vy, wz on /controller/cmd_vel
};

// Target pose in Masha's body frame (ROS REP-103): +X forward, +Y left, +Z up.
// Saveli's AprilTag TF is already in this frame. Cat starts as a pixel and
// the node fills x,y via depth × camera ray × TF.
struct PoseInBase {
  double x{0.0};  // m, +X out the head
  double y{0.0};  // m, +Y left
  double yaw{0.0};
  double u{0.0};  // pixel column, only if has_pixel (cat / debug overlay)
  double v{0.0};  // pixel row
  bool has_pixel{false};
  bool range_ok{true};  // false = bbox-height heuristic, not depth
};

struct TargetHit {
  std::string id;            // "saveli" | "cat" — sticky_id and pick_target key
  std::string display_name;
  std::string spoken_wav;    // absolute path to the NAME clip
  PoseInBase pose;
  double range_m{0.0};       // metres along the camera ray before TF, or hypot(x,y)
  bool pose_in_base{false};  // node must TF the cat before tick() if this is false
  double confidence{1.0};
};

// Gains and limits. C++ defaults are conservative; yaml / launch override them.
// PD:  u = clamp(kp * e + kd * e_dot, ±limit). Deadband zeros a small e.
struct HunterConfig {
  double r_target{0.80};     // desired hypot(x,y) in Follow, metres
  double d_stop{0.55};       // LiDAR closer than this → Stopped
  double d_go{0.70};         // hysteresis: must open past this to leave Stopped
  double lost_timeout{1.5};  // Follow→Hunt; brief TF drops *coast* until then
  double follow_max_s{60.0};
  double search_timeout_s{20.0};  // forget sticky_id; also NAME skip window
  double name_timeout_s{5.0};     // NAME → Follow even if ffplay hangs
  double vx_max{0.05};
  double vy_max{0.04};
  double wz_max{0.30};
  double kp_x{0.35};
  double kd_x{0.05};
  double kp_y{0.50};
  double kd_y{0.05};
  double kp_th{1.20};
  double kd_th{0.10};
  double deadband_x{0.05};   // |x - r_target| < 5 cm → vx = 0
  double deadband_y{0.03};
  double deadband_th{5.0 * kHunterPi / 180.0};
  double theta_crab{15.0 * kHunterPi / 180.0};  // |bearing| must be small to crab
  double y_on{0.06};         // |y| above this latches crab
  double y_off{0.03};        // |y| below this unlatches (hysteresis)
  bool enable_walk{false};
  bool enable_crab{false};
  bool enable_wander{false};
  double wander_vx{0.03};
  double wander_wz{0.25};
  double control_dt{0.05};   // 20 Hz; used as dt for e_dot, not wall time
  double lidar_x{0.102};     // LD19 origin in base_link (metres forward)
  double lidar_y{0.0};
  double lidar_ignore_below{0.20};  // drop own-leg / noise returns
  double lidar_sector_rad{90.0 * kHunterPi / 180.0};
};

// Body-frame velocity command. Maps 1:1 onto geometry_msgs/Twist:
//   linear.x = vx, linear.y = vy (crab), angular.z = wz (yaw rate).
struct TwistBody {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

inline bool twist_is_zero(const TwistBody &t) {
  return t.vx == 0.0 && t.vy == 0.0 && t.wz == 0.0;
}

// Signed errors the PD law consumes. Body frame:
//   r     = hypot(x, y)           distance to target
//   theta = atan2(y, x)           +θ = target is to Masha's LEFT
//   x     = pose.x - r_target     + → too far, command +vx
//   y     = pose.y                + → target on the left, crab +vy
struct FollowErrors {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double r{0.0};
};

struct LidarSector {
  double d_min{0.0};  // +inf if no finite hit in the front sector
  double left_median{0.0};
  double right_median{0.0};
  bool have_hit{false};
};

// Non-owning view of sensor_msgs/LaserScan. The node fills this from the
// message; hunter.cpp never includes ROS headers.
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
  bool pan_head{false};   // Hunt: node runs the triangle sweep on servo 19
  bool play_name{false};  // rising edge: node starts ffplay this tick only
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

// sticky_id (who we named) beats yaml `order` (enabled_targets).
std::optional<TargetHit> pick_target(const std::vector<std::optional<TargetHit>> &hits,
                                     const std::string &sticky_id,
                                     const std::vector<std::string> &order);

class Hunter {
 public:
  explicit Hunter(HunterConfig cfg = {});

  void start();  // Idle → Hunt. Called from ~/start.
  void stop();   // any phase → Idle. Clears sticky / crab / gait latch.
  void notify_name_done();  // WavPlayer finished (or missing file). Name → Follow.
  // Walk watchdog (or any external halt) stood the legs. Next Twist
  // must SelectGait15 again — the controller forgets cmd_gait=5 after stand.
  void notify_halted();
  void set_config(const HunterConfig &cfg) { cfg_ = cfg; }
  const HunterConfig &config() const { return cfg_; }

  // One control step. Call at ~20 Hz. See hunter.cpp for the branch order.
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
  bool crab_{false};              // latched so vy does not chatter on/off
  bool gait15_sent_{false};       // SelectGait15 already published this walk bout
  bool named_this_lock_{false};   // NAME already played for this lock
  bool name_playing_{false};      // waiting for notify_name_done() or timeout
  bool lidar_latched_{false};     // Stopped until d_min > d_go (hysteresis)
  bool ignore_until_sweep_{false};  // after 60 s: finish one pan before re-lock
  bool wall_stood_{false};        // wander: stand one tick before turning
  int wander_kind_{0};            // 0 hold, 1 walk, 2 turn
  double now_{0.0};
  double phase_t_{0.0};           // when we entered the current phase
  double follow_t0_{0.0};         // Follow clock; 60 s is measured from here
  double last_hit_t_{0.0};
  double last_named_t_{-1.0e9};
  double prev_ex_{0.0};           // previous errors, for e_dot = (e - prev) / dt
  double prev_ey_{0.0};
  double prev_eth_{0.0};
  bool have_prev_e_{false};
  std::string sticky_id_;         // keep following the named target, not a new one
  std::string last_named_id_;
  std::string pending_wav_;
  TwistBody last_twist_{};        // coast this Twist during a brief TF miss
};

}  // namespace proud_up
