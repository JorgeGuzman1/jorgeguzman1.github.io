#include "hpp.hpp"

#include "../../../include/EKF/EKF.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aon::hpp {

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

Controller::Controller(const Config& cfg)
  : cfg_(cfg), mp_(cfg.mp_cfg) {}

void Controller::setTrajectory(const Trajectory& traj) {
  traj_ = traj;
  proj_hint_idx_ = 0;
  samp_hint_idx_ = 0;
  mp_.reset();
}

void Controller::clear() {
  traj_.clear();
  proj_hint_idx_ = 0;
  samp_hint_idx_ = 0;
  mp_.reset();
}

double Controller::wrapPi(double a_rad) {
  static constexpr double kPi = 3.14159265358979323846;
  while (a_rad >  kPi) a_rad -= 2.0 * kPi;
  while (a_rad < -kPi) a_rad += 2.0 * kPi;
  return a_rad;
}

double Controller::angleDiff(double a_rad, double b_rad) {
  return wrapPi(a_rad - b_rad);
}

double Controller::projectPoseToS(const Trajectory& traj, const Pose& pose,
                                  double min_seg_len_in, double s_eps_in,
                                  std::size_t* io_hint_idx) {
  if (traj.empty()) return 0.0;
  if (traj.size() == 1) return traj.front().s_in;

  const double px = pose.x_in;
  const double py = pose.y_in;

  const std::size_t nSeg = traj.size() - 1;
  std::size_t start = 0;
  if (io_hint_idx && *io_hint_idx < nSeg) start = *io_hint_idx;

  double bestD2 = std::numeric_limits<double>::infinity();
  double bestS = traj.front().s_in;
  std::size_t bestIdx = 0;

  const double minSeg2 = min_seg_len_in * min_seg_len_in;

  for (std::size_t k = 0; k < nSeg; ++k) {
    const std::size_t i = (start + k) % nSeg;
    const auto& A = traj[i];
    const auto& B = traj[i + 1];

    const double ax = A.x_in, ay = A.y_in;
    const double bx = B.x_in, by = B.y_in;

    const double vx = bx - ax;
    const double vy = by - ay;
    const double segLen2 = vx * vx + vy * vy;
    if (segLen2 < minSeg2) continue;

    const double wx = px - ax;
    const double wy = py - ay;

    double t = (wx * vx + wy * vy) / segLen2;
    t = clampd(t, 0.0, 1.0);

    const double qx = ax + t * vx;
    const double qy = ay + t * vy;

    const double dx = px - qx;
    const double dy = py - qy;
    const double d2 = dx * dx + dy * dy;

    if (d2 < bestD2) {
      bestD2 = d2;
      const double ds = (B.s_in - A.s_in);
      bestS = A.s_in + t * ds;
      bestIdx = i;
    }
  }

  if (io_hint_idx) *io_hint_idx = bestIdx;

  const double s0 = traj.front().s_in;
  const double s1 = traj.back().s_in;
  bestS = clampd(bestS, s0, s1);

  if (std::abs(bestS - s0) < s_eps_in) bestS = s0;
  if (std::abs(bestS - s1) < s_eps_in) bestS = s1;

  return bestS;
}

TrajectorySample Controller::sampleAtS(const Trajectory& traj, double s_target_in,
                                       double s_eps_in, std::size_t* io_hint_idx) {
  TrajectorySample out{};
  if (traj.empty()) return out;
  if (traj.size() == 1) return traj.front();

  const double s0 = traj.front().s_in;
  const double s1 = traj.back().s_in;
  const double s = clampd(s_target_in, s0, s1);

  std::size_t i = 0;
  if (io_hint_idx && *io_hint_idx < traj.size() - 1) i = *io_hint_idx;

  while (i > 0 && traj[i].s_in > s + s_eps_in) --i;
  while (i + 1 < traj.size() && traj[i + 1].s_in < s - s_eps_in) ++i;
  if (i + 1 >= traj.size()) i = traj.size() - 2;

  const auto& A = traj[i];
  const auto& B = traj[i + 1];

  const double ds = (B.s_in - A.s_in);
  double alpha = 0.0;
  if (std::abs(ds) > s_eps_in) alpha = (s - A.s_in) / ds;
  alpha = clampd(alpha, 0.0, 1.0);

  out.x_in = A.x_in + alpha * (B.x_in - A.x_in);
  out.y_in = A.y_in + alpha * (B.y_in - A.y_in);
  out.s_in = s;

  // Metadata interpolation:
  // - heading uses shortest-arc interpolation (wrapPi)
  // - omega_hint is linear
  const double dh = wrapPi(B.h_in - A.h_in);
  out.h_in = wrapPi(A.h_in + alpha * dh);

  if (io_hint_idx) *io_hint_idx = i;
  return out;
}

bool Controller::isFinished(const Trajectory& traj, const Pose& pose, const Config& cfg,
                            double theta_ref_rad) {
  if (traj.empty()) return true;

  const auto& end = traj.back();
  const double dx = end.x_in - pose.x_in;
  const double dy = end.y_in - pose.y_in;
  const double d = std::sqrt(dx * dx + dy * dy);

  if (d > cfg.pos_tol_in) return false;
  if (!cfg.require_heading_for_finish) return true;

  const double e = angleDiff(theta_ref_rad, pose.theta_rad);
  return std::abs(e) <= cfg.heading_tol_rad;
}

StepOutput Controller::step(const StepInput& in) {
  StepOutput out{};
  out.has_path = (traj_.size() >= 2);

  if (!out.has_path) {
    out.cmd = Command{0.0, 0.0, 0.0};
    out.finished = true;
    return out;
  }

  const double s_robot =
    projectPoseToS(traj_, in.pose, cfg_.min_seg_len_in, cfg_.s_eps_in, &proj_hint_idx_);
  out.s_robot_in = s_robot;

  const double s_end = traj_.back().s_in;
  const double remainingDist_in = std::max(0.0, s_end - s_robot);

  const double v_des_ips = mp_.update(remainingDist_in, cfg_.dt_s);

  const double s_target = s_robot + cfg_.lookahead_in;
  out.s_target_in = s_target;

  const TrajectorySample look =
    sampleAtS(traj_, s_target, cfg_.s_eps_in, &samp_hint_idx_);
  out.lookahead = look;

  // Heading reference selection (metadata-first when enabled).
  const double theta_ref_used = cfg_.use_traj_heading ? look.h_in
                                                      : in.theta_ref_rad;

  const double ex_f = look.x_in - in.pose.x_in;
  const double ey_f = look.y_in - in.pose.y_in;

  const double c = std::cos(in.pose.theta_rad);
  const double s = std::sin(in.pose.theta_rad);

  const double ex_r =  c * ex_f + s * ey_f;
  const double ey_r = -s * ex_f + c * ey_f;

  double tx = ex_r;
  double ty = (cfg_.mode == Mode::HDrive) ? ey_r : 0.0;

  const double mag = std::sqrt(tx * tx + ty * ty);
  double ux = 0.0, uy = 0.0;
  if (mag > 1e-9) {
    ux = tx / mag;
    uy = ty / mag;
  }

  double v = std::max(0.0, v_des_ips);
  v = std::min(v, cfg_.v_max_ips);

  Command cmd{};
  cmd.vx_ips = ux * v;
  cmd.vy_ips = (cfg_.mode == Mode::HDrive) ? (uy * v) : 0.0;

  const double h_err = angleDiff(theta_ref_used, in.pose.theta_rad);
  double omega = cfg_.kHeading * h_err;

  omega = clampd(omega, -cfg_.omega_max_rps, cfg_.omega_max_rps);
  cmd.omega_rps = omega;

  out.cmd = cmd;
  out.finished = isFinished(traj_, in.pose, cfg_, theta_ref_used);
  return out;
}

StepOutput Controller::step(const EKF& ekf, double theta_ref_rad) {
  const EKF::State s = ekf.getState();

  StepInput in;
  in.pose.x_in = s.x_in;
  in.pose.y_in = s.y_in;
  in.pose.theta_rad = s.theta_rad;
  in.theta_ref_rad = theta_ref_rad;

  return step(in);
}

StepOutput Controller::step(const EKF& ekf) {
  // When cfg_.use_traj_heading is enabled (default), theta_ref is taken from
  // trajectory metadata and this overload keeps main.cpp clean.
  return step(ekf, 0.0);
}

} // namespace aon::hpp
