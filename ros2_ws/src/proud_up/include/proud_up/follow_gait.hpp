#pragma once

// Follow gait map — variant 3, omnidirectional tripod.
//
// This header has no ROS and no IK. It is the periodic map
//
//   φ ∈ [0, 2π)  →  six foot tips in millimetres
//
// that StepController samples every 20 ms. Vendor kinematics.set_leg_position
// then turns each tip into three joint angles. We never call
// kinematics.set_step_mode(..., gait=5, ...): the .so does not know 5.
//
// Frame (same as move.py / DEFAULT_POSE):
//   +X head, +Y left, +Z up, feet at negative z. Millimetres.
//
// Two clocks (TRAJECTORY.MD / Chapter 9):
//   Cycle — φ keeps rolling. Phase rate is nearly constant. It does *not*
//           go to zero at wrap. A rest-to-rest quintic on the whole step
//           would lurch: slow–fast–slow–stop, then again. That is a creep,
//           not a walk.
//   Swing — while a foot is in the air, progress along its arch is
//           sigma(τ) = 10τ³ − 15τ⁴ + 6τ⁵. sigma-dot = sigma-ddot = 0 at
//           lift and land, so the foot kisses the floor. Vendor linear
//           s = t/T stomps (z-dot ≠ 0 at touchdown).
//
// Stance feet retract at roughly steady speed. They do not rest at wrap.
//
// Tripod groups (MASHA_GAITS.md §3, 1-based leg ids):
//   A (swing while φ ∈ [0, π)):  1 LF, 3 LR, 5 RM
//   B (swing while φ ∈ [π, 2π)): 2 LM, 4 RR, 6 RF
// Arrays in this file are 0-based (leg 1 = index 0).
//
// Body command is a planar twist V = (vx, vy, ωz) in mm/s, mm/s, rad/s.
// A stance foot is fixed on the floor, so in base_link it traces the
// inverse of the body's SE(2) motion. AEP / PEP are the ends of that
// track. Swing is the quintic bridge PEP → AEP, including the lateral
// gap when vy ≠ 0.
//
// The gait does not decide *when* to crab. It only maps the twist it was
// given. Crab policy lives in hunter.hpp.

#include <array>
#include <cmath>
#include <cstddef>

namespace proud_up {
namespace follow_gait {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
inline constexpr double kOmegaEps = 1.0e-6;     // rad/s; below this, SE(2) uses the linear map
inline constexpr double kContactBandMm = 3.0;   // stance feet must stay this close in z
inline constexpr double kDefaultStrideMaxMm = 40.0;
inline constexpr double kDefaultLiftMm = 25.0;
inline constexpr double kDefaultPeriodS = 0.70;
inline constexpr double kDt = 0.02;             // StepController tick, seconds

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Vec3 operator+(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3 operator-(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3 operator*(const Vec3 &a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}

inline double hypot2(double x, double y) { return std::hypot(x, y); }

inline double norm_xy(const Vec3 &p) { return std::hypot(p.x, p.y); }

// Nominal footholds = DEFAULT_POSE in build_in_pose.py (halt gait=-2).
// Recompute AEP/PEP from the *current* six-foot pose when a cycle table
// is rebuilt, so a taller stand still works. These numbers are the
// gtest / dry-run default, not a second source of truth at runtime.
inline constexpr std::array<Vec3, 6> kDefaultPose{{
    {163.6, 140.8, -70.0},    // 0 LF  group A
    {0.0, 183.5, -70.0},      // 1 LM  group B
    {-163.6, 140.8, -70.0},   // 2 LR  group A
    {-163.6, -140.8, -70.0},  // 3 RR  group B
    {0.0, -183.5, -70.0},     // 4 RM  group A
    {163.6, -140.8, -70.0},   // 5 RF  group B
}};

// 0-based indices. A = LF, LR, RM. B = LM, RR, RF.
inline constexpr std::array<int, 3> kGroupA{{0, 2, 4}};
inline constexpr std::array<int, 3> kGroupB{{1, 3, 5}};

inline bool in_group_a(int leg) { return leg == 0 || leg == 2 || leg == 4; }

struct TwistMm {
  double vx{0.0};  // mm/s, +X head
  double vy{0.0};  // mm/s, +Y left
  double wz{0.0};  // rad/s, +CCW
};

struct GaitCommand {
  TwistMm v;
  double lift_mm{kDefaultLiftMm};          // h, lift *above* ground, not stance compression
  double period_s{kDefaultPeriodS};        // T, one full cycle
  double stride_max_mm{kDefaultStrideMaxMm};
  double linear_factor{1.0};               // tape-measure knob; default 1. Do not copy vendor's factor-of-two.
};

struct GaitSample {
  std::array<Vec3, 6> feet{};
  std::array<bool, 6> stance{};  // true = on the ground this sample
};

struct Se2 {
  double tx{0.0};
  double ty{0.0};
  double theta{0.0};
};

struct AepPep {
  Vec3 aep;
  Vec3 pep;
};

// Swing clock. τ in [0, 1] is linear in that foot's air time.
// σ(0)=0, σ(1)=1, σ'(0)=σ'(1)=0, σ''(0)=σ''(1)=0.
// Closed form of σ': 30 τ² (1−τ)² — zero at both ends, which is why
// x-dot and z-dot vanish at lift and plant even on a spatial parabola.
inline double sigma(double tau) {
  const double t = tau < 0.0 ? 0.0 : (tau > 1.0 ? 1.0 : tau);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t2 * t2;
  const double t5 = t4 * t;
  return 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
}

inline double sigma_dot(double tau) {
  const double t = tau < 0.0 ? 0.0 : (tau > 1.0 ? 1.0 : tau);
  const double omt = 1.0 - t;
  return 30.0 * t * t * omt * omt;
}

// Integrate a constant body-frame twist for time t, starting at the
// identity at mid-stance. Lynch & Park: this is exp(ξ̂ t) on SE(2).
//
//   θ(t) = ωz t
//   if |ωz| tiny: trans = (vx t, vy t)
//   else:
//     tx = ( vx sinθ − vy (1 − cosθ) ) / ωz
//     ty = ( vy sinθ + vx (1 − cosθ) ) / ωz
//
// vx, vy in mm/s, t in seconds, result translation in mm.
Se2 se2_exp(double vx, double vy, double wz, double t);

// Rotate (x, y) by yaw α: (x cosα − y sinα, x sinα + y cosα).
inline void rot2(double x, double y, double alpha, double *ox, double *oy) {
  const double c = std::cos(alpha);
  const double s = std::sin(alpha);
  *ox = x * c - y * s;
  *oy = x * s + y * c;
}

// World-fixed foot that sat at body point p0 at mid-stance:
//   p_body(t) = R(−θ(t)) (p0 − trans(t))
Vec3 body_of_world_fixed(const Vec3 &p0, const TwistMm &v, double t);

// AEP = p_body(−Ts/2)  (just landed; start of stance)
// PEP = p_body(+Ts/2)  (about to lift; end of stance)
//
// Sign checks (gtest, not comments-only):
//   vx > 0, vy = ωz = 0  →  AEP of LF is +X of p0
//   vy > 0, vx = ωz = 0  →  AEP of LF is +Y of p0
//   ωz > 0, vx = vy = 0  →  right-side feet (y0 < 0) get extra +X
//
// If |AEP − p0| or |PEP − p0| exceeds stride_max, scale that foot's
// offset down, preserving direction. Vendor cmd_vel_basic_data reports
// AEP offset = vx * T / 2 so AEP−PEP = vx T. Our no-slip with Ts = T/2
// gives AEP−PEP ≈ v_hip * Ts = v_hip * T/2. We do *not* copy that
// factor-of-two. linear_factor (default 1) is the tape-measure knob.
AepPep aep_pep(const Vec3 &p0, const TwistMm &v, double ts, double stride_max_mm,
               double linear_factor = 1.0);

// One 20 ms bead. φ is wrapped into [0, 2π).
//   φ ∈ [0, π):  A swing, B stance
//   φ ∈ [π, 2π): A stance, B swing
// At the two switch samples all six z = z_gnd.
GaitSample sample(double phi, const GaitCommand &cmd,
                  const std::array<Vec3, 6> &nominal = kDefaultPose);

// Equal-mass CoM proxy: mean of the six nominal hips in xy, or the
// body origin if the mean is used as a cheap stand-in. Static support
// contract: the grounded feet's triangle (tripod) or polygon must
// contain this point.
bool point_in_triangle(double px, double py, const Vec3 &a, const Vec3 &b, const Vec3 &c);

bool com_in_support(const GaitSample &s, double com_x = 0.0, double com_y = 0.0);

}  // namespace follow_gait
}  // namespace proud_up
