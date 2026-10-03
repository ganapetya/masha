#include "proud_up/hunter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace proud_up {
namespace {

// Partial sort: put the middle element in place, leave the rest unordered.
// Cheaper than std::sort when we only need the median.
double median_or_zero(std::vector<double> *v) {
  if (v->empty()) {
    return 0.0;
  }
  const std::size_t n = v->size();
  std::nth_element(v->begin(), v->begin() + static_cast<std::ptrdiff_t>(n / 2), v->end());
  return (*v)[n / 2];
}

}  // namespace

const char *hunter_phase_name(HunterPhase p) {
  switch (p) {
    case HunterPhase::Idle:
      return "idle";
    case HunterPhase::Hunt:
      return "hunt";
    case HunterPhase::Name:
      return "name";
    case HunterPhase::Follow:
      return "follow";
    case HunterPhase::Stopped:
      return "stopped";
  }
  return "unknown";
}

// Polar + Cartesian errors in base_link.
// Example: Saveli at (x=1.00, y=0.20), r_target=0.80
//   r     = 1.02 m
//   theta = +11°  (left of the nose)
//   e.x   = +0.20 m  → command +vx (walk forward)
//   e.y   = +0.20 m  → if crab latched, command +vy (sidestep left)
FollowErrors errors_from_pose(const PoseInBase &p, double r_target) {
  FollowErrors e;
  e.r = std::hypot(p.x, p.y);
  e.theta = std::atan2(p.y, p.x);  // +θ = target to Masha's left
  e.x = p.x - r_target;
  e.y = p.y;
  return e;
}

double clamp_val(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// One-axis PD. Derivative damps overshoot (Masha's mass + gait delay).
// Deadband first: a 2 cm error must not keep the legs shuffling.
double pd_axis(double e, double e_dot, double kp, double kd, double deadband, double limit) {
  if (std::fabs(e) < deadband) {
    return 0.0;
  }
  return clamp_val(kp * e + kd * e_dot, -limit, limit);
}

// Project each LD19 return into base_link, keep the front 90° sector.
// The scan is in the lidar frame; lidar_x (~10 cm) is the mount offset
// so "0.55 m in front of Masha" is not "0.55 m in front of the puck".
// The same offset pulls a side ray into the wedge: a return at 53° in
// the lidar frame can sit at 44° from the body, and ±45° still counts
// as "in front". d_min_bearing is that body angle of the closest return.
LidarSector lidar_front(const ScanView &scan, const HunterConfig &cfg) {
  LidarSector out;
  out.d_min = std::numeric_limits<double>::infinity();
  if (scan.ranges == nullptr || scan.n == 0) {
    return out;
  }
  const double half = 0.5 * cfg.lidar_sector_rad;
  std::vector<double> left;
  std::vector<double> right;
  left.reserve(scan.n / 2);
  right.reserve(scan.n / 2);
  for (std::size_t i = 0; i < scan.n; ++i) {
    const double r = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(r) || r < cfg.lidar_ignore_below) {
      continue;
    }
    if (scan.range_max > 0.0 && r > scan.range_max) {
      continue;
    }
    const double ang = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x_b = cfg.lidar_x + r * std::cos(ang);
    const double y_b = cfg.lidar_y + r * std::sin(ang);
    const double bearing = std::atan2(y_b, x_b);
    if (std::fabs(bearing) > half) {
      continue;
    }
    const double d = std::hypot(x_b, y_b);
    out.have_hit = true;
    if (d < out.d_min) {
      out.d_min = d;
      out.d_min_bearing = bearing;
    }
    if (bearing > 0.0) {
      left.push_back(d);
    } else {
      right.push_back(d);
    }
  }
  out.left_median = median_or_zero(&left);
  out.right_median = median_or_zero(&right);
  return out;
}

// Priority: (1) preempt_rank above the sticky lock — equal ranks do not
// steal; yaml order breaks a tie, (2) exclusive_coast with that id missing
// → empty, so a lower plug cannot cut in, (3) sticky id, (4) yaml order,
// (5) any hit. empty = "no detection this tick", not a zero pose.
std::optional<TargetHit> pick_target(const std::vector<std::optional<TargetHit>> &hits,
                                     const std::string &sticky_id,
                                     const std::vector<std::string> &order,
                                     const TargetPolicy &sticky_policy) {
  // No lock yet: only rank > 0 preempts, so two tags still follow yaml order.
  const int floor_rank = sticky_id.empty() ? 0 : sticky_policy.preempt_rank;
  int winning = floor_rank;
  bool preempt = false;
  for (const auto &h : hits) {
    if (h && h->policy.preempt_rank > floor_rank) {
      preempt = true;
      winning = std::max(winning, h->policy.preempt_rank);
    }
  }
  if (preempt) {
    for (const auto &want : order) {
      for (const auto &h : hits) {
        if (h && h->id == want && h->policy.preempt_rank == winning) {
          return h;
        }
      }
    }
    for (const auto &h : hits) {
      if (h && h->policy.preempt_rank == winning) {
        return h;
      }
    }
  }
  if (!sticky_id.empty() && sticky_policy.exclusive_coast) {
    for (const auto &h : hits) {
      if (h && h->id == sticky_id) {
        return h;
      }
    }
    return std::nullopt;
  }
  if (!sticky_id.empty()) {
    for (const auto &h : hits) {
      if (h && h->id == sticky_id) {
        return h;
      }
    }
  }
  for (const auto &want : order) {
    for (const auto &h : hits) {
      if (h && h->id == want) {
        return h;
      }
    }
  }
  for (const auto &h : hits) {
    if (h) {
      return h;
    }
  }
  return std::nullopt;
}

std::optional<TargetHit> confirm_cat_hit(CatConfirmState &state, const std::optional<TargetHit> &hit,
                                        double gate_px, int need) {
  if (need < 1) {
    need = 1;
  }
  if (!hit || !hit->pose.has_pixel) {
    if (state.count <= 0) {
      state = {};
      return std::nullopt;
    }
    // One blur frame while walking used to zero the streak, and the
    // next good frame had to start over. Two empty frames means it left.
    state.misses += 1;
    if (state.misses >= 2) {
      state = {};
    }
    return std::nullopt;
  }
  if (state.count > 0) {
    const double d = std::hypot(hit->pose.u - state.u, hit->pose.v - state.v);
    if (d > gate_px) {
      state.count = 0;
      state.misses = 0;
    }
  }
  state.misses = 0;
  state.count += 1;
  state.u = hit->pose.u;
  state.v = hit->pose.v;
  if (state.count < need) {
    return std::nullopt;
  }
  return hit;
}

LegCommandKind legs_for_metric_target(LegCommandKind legs, const std::optional<TargetHit> &hit) {
  if (legs != LegCommandKind::Twist && legs != LegCommandKind::SelectGait15) {
    return legs;
  }
  if (!hit || !hit->policy.walk_needs_metric) {
    return legs;
  }
  if (hit->pose_in_base && hit->pose.range_ok) {
    return legs;
  }
  return LegCommandKind::None;
}

Hunter::Hunter(HunterConfig cfg) : cfg_(std::move(cfg)) {}

// now_ is 0 until the first tick. Hunt pan timing lives in the node
// (hunt_pan_t0_), not here — the policy only cares about the phase.
void Hunter::start() {
  enter(HunterPhase::Hunt, now_);
}

void Hunter::clear_sticky() {
  sticky_id_.clear();
  sticky_policy_ = {};
}

void Hunter::stop() {
  clear_sticky();
  crab_ = false;
  gait15_sent_ = false;
  lidar_latched_ = false;
  wander_kind_ = 0;
  last_twist_ = {};
  name_playing_ = false;
  greet_loop_ = false;
  greet_silence_open_ = false;
  enter(HunterPhase::Idle, now_);
}

void Hunter::notify_name_done() {
  end_name_clip(now_);
}

void Hunter::notify_name_done(double now_s) {
  end_name_clip(now_s);
}

void Hunter::end_name_clip(double now_s) {
  if (!name_playing_) {
    return;
  }
  name_playing_ = false;
  if (!greet_loop_) {
    return;
  }
  greet_silent_since_ = now_s;
  greet_silence_open_ = true;
}

void Hunter::maybe_regreet(const std::optional<TargetHit> &seen, double now_s, HunterOutput &o) {
  if (o.play_name || name_playing_ || !greet_silence_open_) {
    return;
  }
  if (phase_ == HunterPhase::Idle || phase_ == HunterPhase::Name) {
    return;
  }
  // Tag plugs leave this at 0 and are named once. A vision plug sets
  // the gap on the hit (yaml cat_greet_silence_s for the cat).
  if (!seen || seen->policy.greet_silence_s <= 0.0) {
    return;
  }
  if ((now_s - greet_silent_since_) < seen->policy.greet_silence_s) {
    return;
  }
  // Stay in the current phase. Re-entering Name would halt and restart
  // the follow clock; the repeat is the clip only.
  name_playing_ = true;
  greet_loop_ = true;
  greet_silence_open_ = false;
  named_this_lock_ = true;
  last_named_id_ = seen->id;
  last_named_t_ = now_s;
  o.play_name = true;
}

HunterOutput Hunter::finish_tick(HunterOutput o, const std::optional<TargetHit> &seen, double now_s) {
  maybe_regreet(seen, now_s, o);
  return o;
}

void Hunter::notify_halted() {
  gait15_sent_ = false;
}

// Centralize "what resets when a phase begins". Leaving Follow must
// forget gait15 and the PD history, otherwise the next Follow inherits
// a stale e_dot and the legs lurch.
bool Hunter::name_this_hit(const TargetHit &hit, const std::string &prev_sticky,
                           double now_s) const {
  // A vision plug taking over some other lock always speaks, including
  // inside the 20 s quiet window of an earlier clip of this same id.
  if (hit.policy.rename_on_preempt && !prev_sticky.empty() && prev_sticky != hit.id) {
    return true;
  }
  if (named_this_lock_) {
    return false;
  }
  if (hit.id == last_named_id_ && (now_s - last_named_t_) < cfg_.search_timeout_s) {
    return false;
  }
  return true;
}

void Hunter::arm_name(const TargetHit &hit, double now_s, HunterOutput &o) {
  pending_wav_ = hit.spoken_wav;
  name_playing_ = true;
  greet_silence_open_ = false;
  greet_loop_ = hit.policy.greet_silence_s > 0.0;
  named_this_lock_ = true;
  last_named_id_ = hit.id;
  last_named_t_ = now_s;
  enter(HunterPhase::Name, now_s);
  o.phase = HunterPhase::Name;
  o.legs = LegCommandKind::Halt;
  o.play_name = true;
  o.pan_head = false;
  o.sticky_id = sticky_id_;
  o.crab = false;
}

void Hunter::enter(HunterPhase p, double now_s) {
  phase_ = p;
  phase_t_ = now_s;
  if (p != HunterPhase::Follow) {
    gait15_sent_ = false;
    have_prev_e_ = false;
    crab_ = false;
    last_twist_ = {};
  }
  if (p == HunterPhase::Hunt) {
    wander_kind_ = 0;
  }
}

// Follow law, body frame:
//   1. Face the target (wz from bearing θ).
//   2. Close the range error (vx from x - r_target).
//   3. If already roughly facing it AND offset to the side, crab (vy)
//      instead of spinning. Hysteresis (y_on / y_off) stops chatter.
// Crab with wz still at full gain would yaw while sliding — the 0.3
// scale keeps the nose on Saveli without fighting the sidestep.
TwistBody Hunter::follow_twist(const PoseInBase &pose, double dt) {
  const FollowErrors e = errors_from_pose(pose, cfg_.r_target);
  double ex_dot = 0.0;
  double ey_dot = 0.0;
  double eth_dot = 0.0;
  if (have_prev_e_ && dt > 1.0e-4) {
    ex_dot = (e.x - prev_ex_) / dt;
    ey_dot = (e.y - prev_ey_) / dt;
    eth_dot = (e.theta - prev_eth_) / dt;
  }
  prev_ex_ = e.x;
  prev_ey_ = e.y;
  prev_eth_ = e.theta;
  have_prev_e_ = true;

  TwistBody t;
  t.vx = pd_axis(e.x, ex_dot, cfg_.kp_x, cfg_.kd_x, cfg_.deadband_x, cfg_.vx_max);
  t.wz = pd_axis(e.theta, eth_dot, cfg_.kp_th, cfg_.kd_th, cfg_.deadband_th, cfg_.wz_max);

  if (cfg_.enable_crab) {
    if (std::fabs(e.theta) < cfg_.theta_crab && std::fabs(e.y) > cfg_.y_on) {
      crab_ = true;
    } else if (crab_ && (std::fabs(e.theta) > cfg_.theta_crab || std::fabs(e.y) < cfg_.y_off)) {
      crab_ = false;
    }
  } else {
    crab_ = false;
  }
  if (crab_) {
    t.vy = pd_axis(e.y, ey_dot, cfg_.kp_y, cfg_.kd_y, cfg_.deadband_y, cfg_.vy_max);
    t.wz *= 0.3;
  } else {
    t.vy = 0.0;
  }
  return t;
}

// Hunt default is stand + pan the head. Wander (yaml, default off) adds
// a slow walk into open space. Wall: halt, finish the pan, stand one
// extra tick, SelectGait15, then yaw toward the more-open side.
HunterOutput Hunter::hunt_motion(double d_min, bool pan_sweep_done, double left_open,
                                 double right_open) {
  HunterOutput o;
  o.phase = HunterPhase::Hunt;
  o.pan_head = true;
  o.wander_action = "hold";
  o.legs = LegCommandKind::Halt;
  if (!cfg_.enable_wander || !cfg_.enable_walk) {
    return o;
  }
  const bool wall = std::isfinite(d_min) && d_min < cfg_.d_stop;
  const bool open = !std::isfinite(d_min) || d_min > cfg_.d_go;
  if (wall) {
    if (wander_kind_ != 2) {
      gait15_sent_ = false;
      wall_stood_ = false;
    }
    wander_kind_ = 2;
  } else if (open) {
    wander_kind_ = 1;
    wall_stood_ = false;
  }
  if (wander_kind_ == 2) {
    o.wander_action = "turn";
    o.legs = LegCommandKind::Halt;
    if (!pan_sweep_done) {
      return o;
    }
    if (!wall_stood_) {
      wall_stood_ = true;
      return o;
    }
    if (!gait15_sent_) {
      o.legs = LegCommandKind::SelectGait15;
      gait15_sent_ = true;
      return o;
    }
    o.legs = LegCommandKind::Twist;
    o.twist.wz = (left_open >= right_open) ? cfg_.wander_wz : -cfg_.wander_wz;
    return o;
  }
  if (wander_kind_ == 1) {
    o.wander_action = "walk";
    if (!gait15_sent_) {
      o.legs = LegCommandKind::SelectGait15;
      gait15_sent_ = true;
    } else {
      o.legs = LegCommandKind::Twist;
      o.twist.vx = cfg_.wander_vx;
    }
    return o;
  }
  return o;
}

// One 50 ms step. Branch order is the logic — read top to bottom:
//
//   1. An exclusive coast (vision lock, target missing) drops any other
//      plug's hit. Then stamp last_hit_t_ / sticky_id / sticky_policy.
//   2. Idle: halt, return. start() is the only way out.
//   3. lost = no hit for lost_timeout (1.5 s). Brief gaps are NOT lost.
//      release_sticky_on_lost clears the lock so the next tick may name
//      someone else. A tag lock waits for search_timeout_s instead.
//   4. rename_on_preempt from Follow / Name / Stopped enters NAME.
//      Hunt names in the branch below. No timed look-away.
//   5. Hunt: maybe skip NAME (already named this lock), else NAME or wander.
//      The head pans only while Hunt has no hit.
//   6. Name: freeze until notify_name_done() or name_timeout_s.
//      The timeout leaves Name but leaves name_playing_ set, so a repeating
//      clip's silence clock still starts when the clip actually finishes.
//   7. Follow / Stopped: LiDAR hysteresis, coast on a brief miss, else PD
//      Twist. Zero Twist → Halt, never publish Twist{0,0,0}.
//      lost_timeout with no hit returns to Hunt and pans. A held target
//      stays in Follow; there is no follow clock.
//   8. greet_silence_s > 0 and that quiet has passed: play_name again.
//      finish_tick() does that on the way out. Lost clears the loop.
// Species (Saveli, cat, a later dog or tag robot) is TargetPolicy on the
// hit. This function does not compare id to "cat" or "saveli".
HunterOutput Hunter::tick(double now_s, const std::optional<TargetHit> &hit, double d_min,
                          bool pan_sweep_done, double left_open, double right_open) {
  now_ = now_s;
  const double dt = cfg_.control_dt > 1.0e-4 ? cfg_.control_dt : 0.05;
  HunterOutput o;
  o.d_min = d_min;
  o.sticky_id = sticky_id_;
  o.crab = crab_;

  // Exclusive coast: a vision lock ignores every other plug until
  // lost_timeout. pick_target already returns empty; this keeps a direct
  // tick() caller honest until the release below.
  const std::string prev_sticky = sticky_id_;
  std::optional<TargetHit> seen = hit;
  if (sticky_policy_.exclusive_coast && !prev_sticky.empty() && seen && seen->id != prev_sticky) {
    seen.reset();
  }

  if (seen) {
    last_hit_t_ = now_s;
    sticky_id_ = seen->id;
    sticky_policy_ = seen->policy;
    o.sticky_id = sticky_id_;
    const FollowErrors e = errors_from_pose(seen->pose, cfg_.r_target);
    o.r = e.r;
    o.theta = e.theta;
  }

  if (phase_ == HunterPhase::Idle) {
    o.phase = HunterPhase::Idle;
    o.legs = LegCommandKind::Halt;
    o.pan_head = false;
    return finish_tick(o, seen, now_s);
  }

  const bool lost = !seen && (now_s - last_hit_t_) > cfg_.lost_timeout;
  if (!sticky_id_.empty() && (now_s - last_hit_t_) > cfg_.search_timeout_s) {
    clear_sticky();
    o.sticky_id.clear();
  }
  // Vision coast is over. Drop the hold so the next tick may lock a tag.
  // This tick still has no vision hit (seen was cleared or empty), so the
  // other plug is not adopted until pick_target runs again with an empty
  // sticky. A tag lock does not release here; it waits for search_timeout_s.
  if (lost && sticky_policy_.release_sticky_on_lost) {
    clear_sticky();
    o.sticky_id.clear();
    greet_silence_open_ = false;
  }

  // Higher-rank plug (vision over a tag) beats a lock already in Follow /
  // Name / Stopped and speaks again. Hunt names in the branch below.
  if (seen && seen->policy.rename_on_preempt && seen->id != prev_sticky &&
      phase_ != HunterPhase::Hunt && name_this_hit(*seen, prev_sticky, now_s)) {
    arm_name(*seen, now_s, o);
    return finish_tick(o, seen, now_s);
  }

  if (phase_ == HunterPhase::Hunt) {
    // Pan only while nothing is locked. A hit locks immediately: we do
    // not finish a sweep first, and we do not leave Follow on a clock
    // just to look for someone else.
    if (seen) {
      // Same id inside search_timeout_s: Follow, no new Name phase.
      // A repeating plug that never left still speaks from finish_tick.
      // rename_on_preempt (vision taking a tag lock) plays NAME.
      if (!name_this_hit(*seen, prev_sticky, now_s)) {
        enter(HunterPhase::Follow, now_s);
      } else {
        arm_name(*seen, now_s, o);
        return finish_tick(o, seen, now_s);
      }
    } else {
      o = hunt_motion(d_min, pan_sweep_done, left_open, right_open);
      o.sticky_id = sticky_id_;
      o.d_min = d_min;
      return finish_tick(o, seen, now_s);
    }
  }

  if (phase_ == HunterPhase::Name) {
    o.phase = HunterPhase::Name;
    o.legs = LegCommandKind::Halt;
    o.pan_head = false;
    // notify_name_done() clears name_playing_ when aplay exits. The
    // timeout leaves Name if that never arrives. It must not clear
    // name_playing_: the cat silence clock starts at the real clip end.
    if (!name_playing_ || (now_s - phase_t_) >= cfg_.name_timeout_s) {
      enter(HunterPhase::Follow, now_s);
    } else {
      return finish_tick(o, seen, now_s);
    }
  }

  if (phase_ == HunterPhase::Follow || phase_ == HunterPhase::Stopped) {
    // Hysteresis: enter Stopped at d_stop, leave only after d_go. Without
    // it, a return at 0.55 m would chatter Halt/Twist every other tick.
    // The followed target is not a wall: a return within 20 cm of its
    // range is the target itself, and she keeps closing to r_target.
    const double target_r =
        seen ? std::hypot(seen->pose.x, seen->pose.y) : std::numeric_limits<double>::infinity();
    const bool target_return =
        std::isfinite(d_min) && std::isfinite(target_r) && (d_min + 0.20 >= target_r);
    if (std::isfinite(d_min) && d_min < cfg_.d_stop && !target_return) {
      lidar_latched_ = true;
      enter(HunterPhase::Stopped, now_s);
    }
    if (lidar_latched_ && std::isfinite(d_min) && d_min > cfg_.d_go) {
      lidar_latched_ = false;
      if (seen) {
        enter(HunterPhase::Follow, now_s);
      }
    }

    if (phase_ == HunterPhase::Stopped) {
      o.phase = HunterPhase::Stopped;
      o.legs = LegCommandKind::Halt;
      o.pan_head = false;
      if (!seen && lost) {
        named_this_lock_ = false;
        lidar_latched_ = false;
        enter(HunterPhase::Hunt, now_s);
        o.phase = HunterPhase::Hunt;
        o.pan_head = true;
      }
      return finish_tick(o, seen, now_s);
    }

    if (!seen && lost) {
      named_this_lock_ = false;
      enter(HunterPhase::Hunt, now_s);
      o.phase = HunterPhase::Hunt;
      o.legs = LegCommandKind::Halt;
      o.pan_head = true;
      return finish_tick(o, seen, now_s);
    }

    o.phase = HunterPhase::Follow;
    o.pan_head = false;
    if (!seen) {
      // Brief miss (duplicate prune, motion blur). Hold last Twist so
      // the 0.30 s walk watchdog does not stand the legs, and so Hunt
      // pan does not start. Lost after lost_timeout still goes Hunt.
      o.twist = last_twist_;
      o.crab = crab_;
      if (!cfg_.enable_walk) {
        o.legs = LegCommandKind::None;
        return finish_tick(o, seen, now_s);
      }
      if (twist_is_zero(last_twist_)) {
        o.legs = LegCommandKind::Halt;
        gait15_sent_ = false;
        return finish_tick(o, seen, now_s);
      }
      if (!gait15_sent_) {
        o.legs = LegCommandKind::SelectGait15;
        gait15_sent_ = true;
        return finish_tick(o, seen, now_s);
      }
      o.legs = LegCommandKind::Twist;
      return finish_tick(o, seen, now_s);
    }

    const TwistBody t = follow_twist(seen->pose, dt);
    last_twist_ = t;
    o.twist = t;
    o.crab = crab_;
    // Settled on the standoff. A few centimetres of deadband, not a
    // second stop distance. The reach greeting keys off this flag.
    const double range = std::hypot(seen->pose.x, seen->pose.y);
    o.at_standoff = std::fabs(range - cfg_.r_target) <= cfg_.deadband_x && twist_is_zero(t);
    if (!cfg_.enable_walk) {
      o.legs = LegCommandKind::None;
      return finish_tick(o, seen, now_s);
    }
    // Inside deadband: stand. Next non-zero Twist must SelectGait15 again.
    if (twist_is_zero(t)) {
      o.legs = LegCommandKind::Halt;
      gait15_sent_ = false;
      return finish_tick(o, seen, now_s);
    }
    if (!gait15_sent_) {
      o.legs = LegCommandKind::SelectGait15;
      gait15_sent_ = true;
      return finish_tick(o, seen, now_s);
    }
    o.legs = LegCommandKind::Twist;
    return finish_tick(o, seen, now_s);
  }

  o.phase = phase_;
  o.legs = LegCommandKind::Halt;
  return finish_tick(o, seen, now_s);
}

}  // namespace proud_up
