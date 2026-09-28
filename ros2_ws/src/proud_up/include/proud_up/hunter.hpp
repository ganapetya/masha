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
// A vision target that stays in view is greeted again after its
// TargetPolicy::greet_silence_s of quiet. The clock starts when the clip
// finishes. That repeat sets play_name and does not leave Follow,
// Stopped, or Hunt. Tag targets leave greet_silence_s at 0 and are
// named once per lock.
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
// Name     freeze, play the locked target's clip, then Follow.
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

// How one plug participates in the shared phase machine.
// Hunter::tick never branches on the id string. Saveli stamps
// tag_target_policy(); the cat stamps vision_target_policy(). A dog is
// another vision plug (raise preempt_rank if it should beat the cat).
// Another robot's AprilTag is another tag plug with its own id and wav.
struct TargetPolicy {
  // Strictly above the sticky lock's rank: this hit wins pick_target and
  // can re-enter Name. 0 = tag. 1 = vision. Equal ranks do not steal;
  // sticky, then enabled_targets order, decides.
  int preempt_rank{0};
  // While this id is sticky and missing this frame, pick_target returns
  // empty so a lower-rank hit cannot cut in. The brain drops the lock
  // after lost_timeout when release_sticky_on_lost is set.
  bool exclusive_coast{false};
  // lost_timeout clears sticky_id and the greet-silence clock, so the
  // next tick may lock someone else.
  bool release_sticky_on_lost{false};
  // Switching onto this id from a different lock always plays NAME,
  // including inside search_timeout_s of an earlier clip of the same id.
  bool rename_on_preempt{false};
  // Seconds of quiet after the clip ends before play_name again, while
  // this sighting lasts. 0 names once per lock.
  double greet_silence_s{0.0};
  // Twist and SelectGait15 need pose_in_base and range_ok. A pixel-only
  // vision hit may still NAME and gaze.
  bool walk_needs_metric{false};
  // A fresh pixel aims the head (servo 19/22) instead of the hunt pan.
  bool gaze_on_pixel{false};
};

// Fiducial / AprilTag. Name once. A frame with no tag may adopt another
// hit. The pose is already metric. The head holds; it does not gaze.
inline TargetPolicy tag_target_policy() {
  return TargetPolicy{};
}

// Camera class (cat, and the same shape for a later dog). Outranks a tag
// lock. A miss blocks other targets until lost_timeout. The clip repeats
// after greet_silence_s. Pixel-only poses do not walk. A pixel aims the head.
inline TargetPolicy vision_target_policy(double greet_silence_s = 5.0) {
  TargetPolicy p;
  p.preempt_rank = 1;
  p.exclusive_coast = true;
  p.release_sticky_on_lost = true;
  p.rename_on_preempt = true;
  p.greet_silence_s = greet_silence_s;
  p.walk_needs_metric = true;
  p.gaze_on_pixel = true;
  return p;
}

// Target pose in Masha's body frame (ROS REP-103): +X forward, +Y left, +Z up.
// A tag TF is already in this frame. A vision hit starts as a pixel and
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
  std::string id;            // plug key: "saveli", "cat", or the next one
  std::string display_name;
  std::string spoken_wav;    // absolute path to the NAME clip
  TargetPolicy policy;       // set by the plug; tick reads this, not id
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
  double name_timeout_s{8.0};     // NAME → Follow even if aplay hangs
  // Node copies this onto the cat plug's TargetPolicy::greet_silence_s.
  // tick() reads the hit, not this field. Measured from clip end.
  double cat_greet_silence_s{5.0};
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
  bool play_name{false};  // rising edge: node starts the NAME clip this tick only
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

// Priority: (1) a hit whose preempt_rank is above the sticky lock,
// (2) exclusive_coast and that id is missing → empty,
// (3) sticky_id, (4) yaml order, (5) any hit.
// sticky_policy is the lock from the previous tick (default: a tag).
std::optional<TargetHit> pick_target(const std::vector<std::optional<TargetHit>> &hits,
                                     const std::string &sticky_id,
                                     const std::vector<std::string> &order,
                                     const TargetPolicy &sticky_policy = {});

// Class-15 centres before the node offers the hit to Hunter.
// One YOLO frame at conf 0.45 is enough to false-NAME a chair, so a
// single frame never counts. Two winning frames do. A third empty
// frame used to wipe a cat that was already in the picture.
inline constexpr int kCatConfirmTicks = 2;

struct CatConfirmState {
  int count{0};
  int misses{0};
  double u{0.0};
  double v{0.0};
};

// `need` centres in a row, each within gate_px of the previous. One empty
// frame keeps the streak (the score flickers while the head or the body
// moves). A second empty frame, or a jump, clears it. Returns nullopt
// until the streak is full.
std::optional<TargetHit> confirm_cat_hit(CatConfirmState &state,
                                        const std::optional<TargetHit> &hit, double gate_px,
                                        int need = kCatConfirmTicks);

// Twist and SelectGait15 need a metric base pose when the hit's policy
// says walk_needs_metric. A pixel, or a bbox-height guess (range_ok
// false), may still NAME and gaze. Those leg commands become None so
// Follow does not walk toward (0, 0). A tag pose is not gated.
LegCommandKind legs_for_metric_target(LegCommandKind legs, const std::optional<TargetHit> &hit);

class Hunter {
 public:
  explicit Hunter(HunterConfig cfg = {});

  void start();  // Idle → Hunt. Called from ~/start.
  void stop();   // any phase → Idle. Clears sticky / crab / gait latch.
  void notify_name_done();  // clip finished; silence clock uses the last tick time
  void notify_name_done(double now_s);  // same, stamped on the node's clock
  // True from play_name until notify_name_done(). The Name-phase timeout
  // does not clear this: aplay may still be draining.
  bool name_playing() const { return name_playing_; }
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
  // Policy of the sticky lock. Default-constructed while nothing is locked,
  // which is tag_target_policy() (rank 0, no exclusive coast).
  const TargetPolicy &sticky_policy() const { return sticky_policy_; }

 private:
  void enter(HunterPhase p, double now_s);
  // rename_on_preempt from a different lock always speaks. Same id inside
  // search_timeout_s does not. named_this_lock_ skips a second clip.
  bool name_this_hit(const TargetHit &hit, const std::string &prev_sticky, double now_s) const;
  void arm_name(const TargetHit &hit, double now_s, HunterOutput &o);
  // Clip actually finished (player exit, or a missing file). Opens the
  // greet-silence clock when this lock repeats. The Name-phase timeout
  // must not call this.
  void end_name_clip(double now_s);
  // Same sighting, greet_silence_s of quiet has passed. Sets play_name.
  // Does not change phase or legs. A tag (greet_silence_s == 0) never hits.
  void maybe_regreet(const std::optional<TargetHit> &seen, double now_s, HunterOutput &o);
  void clear_sticky();
  HunterOutput finish_tick(HunterOutput o, const std::optional<TargetHit> &seen, double now_s);
  TwistBody follow_twist(const PoseInBase &pose, double dt);
  HunterOutput hunt_motion(double d_min, bool pan_sweep_done, double left_open,
                           double right_open);

  HunterConfig cfg_;
  HunterPhase phase_{HunterPhase::Idle};
  bool crab_{false};              // latched so vy does not chatter on/off
  bool gait15_sent_{false};       // SelectGait15 already published this walk bout
  bool named_this_lock_{false};   // NAME already played for this lock
  bool name_playing_{false};      // clip started, notify_name_done() not yet
  bool greet_loop_{false};        // this lock repeats its clip after silence
  bool greet_silence_open_{false};  // repeating clip finished; sighting not lost
  double greet_silent_since_{0.0};  // when that clip finished
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
  TargetPolicy sticky_policy_{};  // that lock's policy; survives a missed frame
  std::string last_named_id_;
  std::string pending_wav_;
  TwistBody last_twist_{};        // coast this Twist during a brief TF miss
};

}  // namespace proud_up
