#pragma once

// Reach-and-grab greeting. No ROS. The hunter node plays this once when
// Follow has settled at r_target (15 cm). A wall stop does not start it.
//
// The camera sits on the arm, so "neck toward the target" is servo 20
// further out, servo 22 a little lower, and servo 19 turned by the
// target bearing. Servo 24 is the gripper. Two closes, then the whole
// arm returns to the hunter rest pose and stays there while she remains
// at the standoff.

#include <algorithm>

namespace proud_up {

struct ReachArm {
  float id19{500.0f};
  float id20{810.0f};
  float id21{180.0f};
  float id22{117.0f};
  float id23{500.0f};
  float id24{500.0f};
  double duration_s{0.8};
};

// Idle   not at the standoff
// Reach  arm out, gripper open
// Close1 first grab
// Open1
// Close2 second grab
// Open2  release
// Home   back to the rest pose
// Hold   rest pose held; do not gaze or pan until she leaves the standoff
enum class ReachPhase { Idle, Reach, Close1, Open1, Close2, Open2, Home, Hold };

struct ReachGreet {
  ReachPhase phase{ReachPhase::Idle};
  double phase_t{0.0};
};

struct ReachGreetOutput {
  bool own_arm{false};  // skip hunt pan and gaze while this is set
  bool publish{false};  // this tick starts one bus move
  ReachArm arm{};
};

inline float clamp_reach_pulse(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

// Gripper travel on this arm is about ±0.7 rad. 1000 pulses span 240°,
// so ±150 pulses from the 500 rest stays inside that stop.
inline constexpr float kGripperOpen = 350.0f;
inline constexpr float kGripperClose = 650.0f;

inline ReachArm reach_toward(const ReachArm &home, float pan_pulse) {
  ReachArm a = home;
  a.id19 = pan_pulse;
  a.id20 = clamp_reach_pulse(home.id20 + 100.0f, 0.0f, 960.0f);
  a.id22 = clamp_reach_pulse(home.id22 - 25.0f, 80.0f, 900.0f);
  a.id24 = kGripperOpen;
  a.duration_s = 0.8;
  return a;
}

inline ReachArm gripper_at(const ReachArm &reached, bool closed) {
  ReachArm a = reached;
  a.id24 = closed ? kGripperClose : kGripperOpen;
  a.duration_s = 0.40;
  return a;
}

// Call once per control tick. now_s is the same clock as the hunter.
inline ReachGreetOutput advance_reach_greet(ReachGreet &g, bool at_standoff, double now_s,
                                            const ReachArm &home, float pan_pulse) {
  ReachGreetOutput out;
  const ReachArm reached = reach_toward(home, pan_pulse);

  auto go = [&](ReachPhase phase, const ReachArm &arm) {
    g.phase = phase;
    g.phase_t = now_s;
    out.own_arm = true;
    out.publish = true;
    out.arm = arm;
  };

  if (!at_standoff) {
    if (g.phase != ReachPhase::Idle && g.phase != ReachPhase::Hold) {
      ReachArm back = home;
      back.duration_s = 0.6;
      out.publish = true;
      out.arm = back;
    }
    g.phase = ReachPhase::Idle;
    return out;
  }

  if (g.phase == ReachPhase::Idle) {
    go(ReachPhase::Reach, reached);
    return out;
  }
  if (g.phase == ReachPhase::Hold) {
    out.own_arm = true;
    return out;
  }

  const double held = now_s - g.phase_t;
  switch (g.phase) {
    case ReachPhase::Reach:
      if (held >= 0.8) {
        go(ReachPhase::Close1, gripper_at(reached, true));
      } else {
        out.own_arm = true;
      }
      break;
    case ReachPhase::Close1:
      if (held >= 0.40) {
        go(ReachPhase::Open1, gripper_at(reached, false));
      } else {
        out.own_arm = true;
      }
      break;
    case ReachPhase::Open1:
      if (held >= 0.40) {
        go(ReachPhase::Close2, gripper_at(reached, true));
      } else {
        out.own_arm = true;
      }
      break;
    case ReachPhase::Close2:
      if (held >= 0.40) {
        go(ReachPhase::Open2, gripper_at(reached, false));
      } else {
        out.own_arm = true;
      }
      break;
    case ReachPhase::Open2:
      if (held >= 0.40) {
        ReachArm back = home;
        back.duration_s = 0.8;
        go(ReachPhase::Home, back);
      } else {
        out.own_arm = true;
      }
      break;
    case ReachPhase::Home:
      if (held >= 0.8) {
        g.phase = ReachPhase::Hold;
      }
      out.own_arm = true;
      break;
    default:
      break;
  }
  return out;
}

}  // namespace proud_up
