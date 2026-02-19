#pragma once

#include <cstddef>
#include <vector>

#include "aon/controls/s-curve-profile.hpp"

class EKF;

namespace aon::hpp {

/**
 * @brief Drivetrain kinematic mode for geometric command generation.
 *
 * Tank: outputs forward velocity only (vy = 0).
 * HDrive: outputs forward and lateral velocities (vx, vy).
 */
enum class Mode { Tank, HDrive };

/**
 * @brief Robot pose in the global (field) frame.
 *
 * Units:
 *  - x_in, y_in: inches
 *  - theta_rad: radians (CCW positive)
 */
struct Pose {
  double x_in = 0.0;
  double y_in = 0.0;
  double theta_rad = 0.0;
};

/**
 * @brief Robot-centric velocity command produced by the controller.
 *
 * Coordinate convention:
 *  - vx_ips: robot-forward (+X)
 *  - vy_ips: robot-right   (+Y) (used only for Mode::HDrive)
 *  - omega_rps: positive CCW yaw rate
 *
 * Units:
 *  - vx_ips, vy_ips: inches per second
 *  - omega_rps: radians per second
 */
struct Command {
  double vx_ips = 0.0;
  double vy_ips = 0.0;
  double omega_rps = 0.0;
};

/**
 * @brief A trajectory sample with arc-length parameterization.
 *
 * The controller assumes s_in is non-decreasing along the vector.
 *
 * Units:
 *  - x_in, y_in: inches
 *  - s_in: inches (arc-length from trajectory start)
 *  - h_in: radians (CCW positive)
 *  - omega_hint_rps: radians per second (CCW positive)
 */
struct TrajectorySample {
  double x_in = 0.0;
  double y_in = 0.0;
  double h_in = 0.0;
  double s_in = 0.0;
};

using Trajectory = std::vector<TrajectorySample>;

/**
 * @brief Controller input for a single deterministic step.
 *
 * theta_ref_rad is used only if cfg.use_traj_heading == false.
 */
struct StepInput {
  Pose pose;
  double theta_ref_rad = 0.0;
};

/**
 * @brief Controller output for a single step.
 *
 * - has_path: indicates a valid trajectory is loaded
 * - finished: indicates position (and optionally heading) is within tolerances
 * - cmd: robot-centric command (ips, rps)
 * - s_robot_in: projection of the robot onto the trajectory arc-length
 * - s_target_in: lookahead arc-length (s_robot + lookahead)
 * - lookahead: sampled point at s_target_in (includes heading metadata)
 */
struct StepOutput {
  bool has_path = false;
  bool finished = false;
  Command cmd{};
  double s_robot_in = 0.0;
  double s_target_in = 0.0;
  TrajectorySample lookahead{};
};

/**
 * @brief Configuration for Holonomic Pure Pursuit with integrated jerk-limited profiling.
 */
struct Config {
  Mode mode = Mode::HDrive;

  double lookahead_in = 12.0;

  double min_seg_len_in = 0.25;
  double s_eps_in = 1e-6;

  double v_max_ips = 60.0;

  double kHeading = 1.5;
  double omega_max_rps = 6.0;

  /**
   * @brief Use trajectory heading metadata as the heading reference.
   *
   * When true, the controller uses lookahead.h_in as theta_ref.
   * This keeps main.cpp clean and makes yaw coherent with the path.
   */
  bool use_traj_heading = true;

  double pos_tol_in = 2.0;
  bool require_heading_for_finish = false;
  double heading_tol_rad = 5.0 * 3.14159265358979323846 / 180.0;

  MotionProfileConfig mp_cfg{};

  double dt_s = 0.02;
};

class Controller {
public:
  explicit Controller(const Config& cfg);

  void setTrajectory(const Trajectory& traj);
  void clear();

  StepOutput step(const StepInput& in);

  StepOutput step(const EKF& ekf, double theta_ref_rad);

  /**
   * @brief Convenience overload that uses trajectory heading metadata.
   *
   * If cfg.use_traj_heading is true (default), no external theta_ref is required:
   *   auto out = hpp.step(ekf);
   */
  StepOutput step(const EKF& ekf);

private:
  static double wrapPi(double a_rad);
  static double angleDiff(double a_rad, double b_rad);

  static double projectPoseToS(const Trajectory& traj, const Pose& pose,
                              double min_seg_len_in, double s_eps_in,
                              std::size_t* io_hint_idx);

  static TrajectorySample sampleAtS(const Trajectory& traj, double s_target_in,
                                    double s_eps_in, std::size_t* io_hint_idx);

  static bool isFinished(const Trajectory& traj, const Pose& pose, const Config& cfg,
                         double theta_ref_rad);

private:
  Config cfg_;
  Trajectory traj_;

  std::size_t proj_hint_idx_ = 0;
  std::size_t samp_hint_idx_ = 0;

  MotionProfile mp_;
};

} // namespace aon::hpp
