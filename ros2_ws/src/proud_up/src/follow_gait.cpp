#include "proud_up/follow_gait.hpp"

#include <algorithm>

namespace proud_up {
namespace follow_gait {
namespace {

double wrap_phi(double phi) {
  if (!std::isfinite(phi)) {
    return 0.0;
  }
  phi = std::fmod(phi, kTwoPi);
  if (phi < 0.0) {
    phi += kTwoPi;
  }
  return phi;
}

void clamp_stride(Vec3 *end, const Vec3 &p0, double stride_max_mm) {
  const double dx = end->x - p0.x;
  const double dy = end->y - p0.y;
  const double d = std::hypot(dx, dy);
  if (d <= stride_max_mm || d < 1.0e-9) {
    return;
  }
  const double s = stride_max_mm / d;
  end->x = p0.x + dx * s;
  end->y = p0.y + dy * s;
}

}  // namespace

Se2 se2_exp(double vx, double vy, double wz, double t) {
  Se2 out;
  out.theta = wz * t;
  if (std::fabs(wz) < kOmegaEps) {
    // Pure translation. The exact exponential collapses to v t.
    out.tx = vx * t;
    out.ty = vy * t;
    return out;
  }
  const double th = out.theta;
  const double s = std::sin(th);
  const double omc = 1.0 - std::cos(th);
  out.tx = (vx * s - vy * omc) / wz;
  out.ty = (vy * s + vx * omc) / wz;
  return out;
}

Vec3 body_of_world_fixed(const Vec3 &p0, const TwistMm &v, double t) {
  const Se2 g = se2_exp(v.vx, v.vy, v.wz, t);
  Vec3 out;
  rot2(p0.x - g.tx, p0.y - g.ty, -g.theta, &out.x, &out.y);
  out.z = p0.z;
  return out;
}

AepPep aep_pep(const Vec3 &p0, const TwistMm &v, double ts, double stride_max_mm,
               double linear_factor) {
  TwistMm scaled = v;
  scaled.vx *= linear_factor;
  scaled.vy *= linear_factor;
  // ωz is already a body rate; linear_factor is a tape-measure scale on
  // translation. Do not scale yaw or the no-slip rotation term flips.

  AepPep out;
  const double half = 0.5 * ts;
  out.aep = body_of_world_fixed(p0, scaled, -half);
  out.pep = body_of_world_fixed(p0, scaled, +half);
  out.aep.z = p0.z;
  out.pep.z = p0.z;
  clamp_stride(&out.aep, p0, stride_max_mm);
  clamp_stride(&out.pep, p0, stride_max_mm);
  return out;
}

GaitSample sample(double phi, const GaitCommand &cmd, const std::array<Vec3, 6> &nominal) {
  GaitSample out;
  phi = wrap_phi(phi);
  const double T = cmd.period_s > 1.0e-3 ? cmd.period_s : kDefaultPeriodS;
  const double Ts = 0.5 * T;
  const double h = cmd.lift_mm;
  const bool a_swing = phi < kPi;
  // Group A swing: τ = φ / π. Group B swing: τ = (φ − π) / π.
  const double tau_lin = a_swing ? (phi / kPi) : ((phi - kPi) / kPi);
  const double sig = sigma(tau_lin);

  for (int leg = 0; leg < 6; ++leg) {
    const Vec3 &p0 = nominal[static_cast<std::size_t>(leg)];
    const AepPep ends = aep_pep(p0, cmd.v, Ts, cmd.stride_max_mm, cmd.linear_factor);
    const bool this_a = in_group_a(leg);
    const bool swinging = this_a ? a_swing : !a_swing;
    out.stance[static_cast<std::size_t>(leg)] = !swinging;

    if (swinging) {
      // xy rides the quintic from PEP (lift) to AEP (land), including vy.
      // z is a spatial parabola in sigma, so dz/dsigma ≠ 0 at the ends,
      // but sigma-dot is 0 there, so z-dot is ~0. That is the whole
      // point versus vendor s = t/T.
      const double o = 1.0 - sig;
      out.feet[static_cast<std::size_t>(leg)].x = o * ends.pep.x + sig * ends.aep.x;
      out.feet[static_cast<std::size_t>(leg)].y = o * ends.pep.y + sig * ends.aep.y;
      out.feet[static_cast<std::size_t>(leg)].z = p0.z + 4.0 * sig * (1.0 - sig) * h;
    } else {
      // Stance: linear in cycle time, not quintic. The body must keep
      // moving. τ_s = 0 at landing (AEP), 1 at lift (PEP).
      const double tau_s = tau_lin;
      const double o = 1.0 - tau_s;
      out.feet[static_cast<std::size_t>(leg)].x = o * ends.aep.x + tau_s * ends.pep.x;
      out.feet[static_cast<std::size_t>(leg)].y = o * ends.aep.y + tau_s * ends.pep.y;
      out.feet[static_cast<std::size_t>(leg)].z = p0.z;
    }
  }
  return out;
}

bool point_in_triangle(double px, double py, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
  // Barycentric, inclusive of edges. A point on a stance edge still
  // counts as supported — the discrete 20 ms tick lands there at the
  // A/B switch.
  const double v0x = c.x - a.x;
  const double v0y = c.y - a.y;
  const double v1x = b.x - a.x;
  const double v1y = b.y - a.y;
  const double v2x = px - a.x;
  const double v2y = py - a.y;
  const double dot00 = v0x * v0x + v0y * v0y;
  const double dot01 = v0x * v1x + v0y * v1y;
  const double dot02 = v0x * v2x + v0y * v2y;
  const double dot11 = v1x * v1x + v1y * v1y;
  const double dot12 = v1x * v2x + v1y * v2y;
  const double den = dot00 * dot11 - dot01 * dot01;
  if (std::fabs(den) < 1.0e-12) {
    return false;
  }
  const double u = (dot11 * dot02 - dot01 * dot12) / den;
  const double v = (dot00 * dot12 - dot01 * dot02) / den;
  return u >= -1.0e-9 && v >= -1.0e-9 && (u + v) <= 1.0 + 1.0e-9;
}

bool com_in_support(const GaitSample &s, double com_x, double com_y) {
  Vec3 tri[3];
  int n = 0;
  for (int i = 0; i < 6; ++i) {
    if (s.stance[static_cast<std::size_t>(i)]) {
      if (n < 3) {
        tri[n] = s.feet[static_cast<std::size_t>(i)];
      }
      ++n;
    }
  }
  if (n < 3) {
    return false;
  }
  // Classic tripod: exactly three down. Wave would have five; we only
  // implement variant 3 here.
  return point_in_triangle(com_x, com_y, tri[0], tri[1], tri[2]);
}

}  // namespace follow_gait
}  // namespace proud_up
