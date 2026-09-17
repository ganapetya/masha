#include "proud_up/hunter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace proud_up {
namespace {

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

double pd_axis(double e, double e_dot, double kp, double kd, double deadband, double limit) {
  if (std::fabs(e) < deadband) {
    return 0.0;
  }
  return clamp_val(kp * e + kd * e_dot, -limit, limit);
}

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
    out.d_min = std::min(out.d_min, d);
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

std::optional<TargetHit> pick_target(const std::vector<std::optional<TargetHit>> &hits,
                                     const std::string &sticky_id,
                                     const std::vector<std::string> &order) {
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

Hunter::Hunter(HunterConfig cfg) : cfg_(std::move(cfg)) {}

void Hunter::start() {
  enter(HunterPhase::Hunt, now_);
}

void Hunter::stop() {
  sticky_id_.clear();
  crab_ = false;
  gait15_sent_ = false;
  lidar_latched_ = false;
  wander_kind_ = 0;
  last_twist_ = {};
  enter(HunterPhase::Idle, now_);
}

void Hunter::notify_name_done() {
  name_playing_ = false;
}

void Hunter::notify_halted() {
  gait15_sent_ = false;
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
  if (p == HunterPhase::Follow) {
    follow_t0_ = now_s;
  }
  if (p == HunterPhase::Hunt) {
    wander_kind_ = 0;
  }
}

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

HunterOutput Hunter::tick(double now_s, const std::optional<TargetHit> &hit, double d_min,
                          bool pan_sweep_done, double left_open, double right_open) {
  now_ = now_s;
  const double dt = cfg_.control_dt > 1.0e-4 ? cfg_.control_dt : 0.05;
  HunterOutput o;
  o.d_min = d_min;
  o.sticky_id = sticky_id_;
  o.crab = crab_;

  if (hit) {
    last_hit_t_ = now_s;
    sticky_id_ = hit->id;
    o.sticky_id = sticky_id_;
    const FollowErrors e = errors_from_pose(hit->pose, cfg_.r_target);
    o.r = e.r;
    o.theta = e.theta;
  }

  if (phase_ == HunterPhase::Idle) {
    o.phase = HunterPhase::Idle;
    o.legs = LegCommandKind::Halt;
    o.pan_head = false;
    return o;
  }

  const bool lost = !hit && (now_s - last_hit_t_) > cfg_.lost_timeout;
  if (!sticky_id_.empty() && (now_s - last_hit_t_) > cfg_.search_timeout_s) {
    sticky_id_.clear();
    o.sticky_id.clear();
  }

  if (phase_ == HunterPhase::Hunt) {
    if (ignore_until_sweep_ && !pan_sweep_done) {
      o = hunt_motion(d_min, pan_sweep_done, left_open, right_open);
      o.sticky_id = sticky_id_;
      o.d_min = d_min;
      return o;
    }
    if (ignore_until_sweep_ && pan_sweep_done) {
      ignore_until_sweep_ = false;
    }
    if (hit) {
      const bool skip_name =
          named_this_lock_ ||
          (hit->id == last_named_id_ && (now_s - last_named_t_) < cfg_.search_timeout_s);
      if (skip_name) {
        enter(HunterPhase::Follow, now_s);
      } else {
        pending_wav_ = hit->spoken_wav;
        name_playing_ = true;
        named_this_lock_ = true;
        last_named_id_ = hit->id;
        last_named_t_ = now_s;
        enter(HunterPhase::Name, now_s);
        o.phase = HunterPhase::Name;
        o.legs = LegCommandKind::Halt;
        o.play_name = true;
        o.pan_head = false;
        return o;
      }
    } else {
      o = hunt_motion(d_min, pan_sweep_done, left_open, right_open);
      o.sticky_id = sticky_id_;
      o.d_min = d_min;
      return o;
    }
  }

  if (phase_ == HunterPhase::Name) {
    o.phase = HunterPhase::Name;
    o.legs = LegCommandKind::Halt;
    o.pan_head = false;
    if (!name_playing_ || (now_s - phase_t_) >= cfg_.name_timeout_s) {
      name_playing_ = false;
      enter(HunterPhase::Follow, now_s);
    } else {
      return o;
    }
  }

  if (phase_ == HunterPhase::Follow || phase_ == HunterPhase::Stopped) {
    const double left = cfg_.follow_max_s - (now_s - follow_t0_);
    o.follow_left_s = left;

    if (left <= 0.0) {
      named_this_lock_ = false;
      ignore_until_sweep_ = true;
      enter(HunterPhase::Hunt, now_s);
      o.phase = HunterPhase::Hunt;
      o.legs = LegCommandKind::Halt;
      o.pan_head = true;
      o.wander_action = "hold";
      return o;
    }

    if (std::isfinite(d_min) && d_min < cfg_.d_stop) {
      lidar_latched_ = true;
      enter(HunterPhase::Stopped, now_s);
    }
    if (lidar_latched_ && std::isfinite(d_min) && d_min > cfg_.d_go) {
      lidar_latched_ = false;
      if (hit) {
        enter(HunterPhase::Follow, now_s);
        follow_t0_ = now_s - (cfg_.follow_max_s - left);
      }
    }

    if (phase_ == HunterPhase::Stopped) {
      o.phase = HunterPhase::Stopped;
      o.legs = LegCommandKind::Halt;
      o.pan_head = false;
      if (!hit && lost) {
        named_this_lock_ = false;
        lidar_latched_ = false;
        enter(HunterPhase::Hunt, now_s);
        o.phase = HunterPhase::Hunt;
        o.pan_head = true;
      }
      return o;
    }

    if (!hit && lost) {
      named_this_lock_ = false;
      enter(HunterPhase::Hunt, now_s);
      o.phase = HunterPhase::Hunt;
      o.legs = LegCommandKind::Halt;
      o.pan_head = true;
      return o;
    }

    o.phase = HunterPhase::Follow;
    o.pan_head = false;
    if (!hit) {
      // Brief miss (duplicate prune, motion blur). Hold last Twist so
      // the 0.30 s walk watchdog does not stand the legs, and so Hunt
      // pan does not start. Lost after lost_timeout still goes Hunt.
      o.twist = last_twist_;
      o.crab = crab_;
      if (!cfg_.enable_walk) {
        o.legs = LegCommandKind::None;
        return o;
      }
      if (twist_is_zero(last_twist_)) {
        o.legs = LegCommandKind::Halt;
        gait15_sent_ = false;
        return o;
      }
      if (!gait15_sent_) {
        o.legs = LegCommandKind::SelectGait15;
        gait15_sent_ = true;
        return o;
      }
      o.legs = LegCommandKind::Twist;
      return o;
    }

    const TwistBody t = follow_twist(hit->pose, dt);
    last_twist_ = t;
    o.twist = t;
    o.crab = crab_;
    if (!cfg_.enable_walk) {
      o.legs = LegCommandKind::None;
      return o;
    }
    if (twist_is_zero(t)) {
      o.legs = LegCommandKind::Halt;
      gait15_sent_ = false;
      return o;
    }
    if (!gait15_sent_) {
      o.legs = LegCommandKind::SelectGait15;
      gait15_sent_ = true;
      return o;
    }
    o.legs = LegCommandKind::Twist;
    return o;
  }

  o.phase = phase_;
  o.legs = LegCommandKind::Halt;
  return o;
}

}  // namespace proud_up
