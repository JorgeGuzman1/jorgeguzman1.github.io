#include "aon/tank-drive/tank-drive.hpp"
#include "aon/constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace aon {

// ==============================
// TankDrive
// ==============================

TankDrive::TankDrive(const std::initializer_list<okapi::Motor> &leftPorts,
                     const std::initializer_list<okapi::Motor> &rightPorts)
    : leftMotors_(leftPorts),
      rightMotors_(rightPorts) {}

void TankDrive::bindMotionProfileConfig(const MotionProfileConfig &cfg) {
  mp_cfg_ = &cfg;
}

void TankDrive::setKinematics(const TankKinematicsConfig &cfg) {
  kin_ = cfg;
}

double TankDrive::clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

double TankDrive::maxMotorRpmLimit() const {
  return (mp_cfg_ != nullptr) ? std::abs(mp_cfg_->max_velocity_rpm)
                              : std::abs(MAX_RPM);
}

double TankDrive::ipsToMotorRpm(double v_ips) const {
  const double D = kin_.wheel_diam_in;
  const double pi = std::acos(-1.0);
  const double circ = pi * D;
  if (circ <= 0.0) return 0.0;

  const double wheel_rpm = (v_ips / circ) * 60.0;

  // MotionProfileConfig convention:
  //   wheel_rpm = motor_rpm * motor_to_drive_ratio
  // => motor_rpm = wheel_rpm / motor_to_drive_ratio
  const double ratio = (mp_cfg_ != nullptr) ? mp_cfg_->motor_to_drive_ratio : 1.0;
  if (std::abs(ratio) <= 1e-9) return 0.0;

  return wheel_rpm / ratio;
}

void TankDrive::setWheelVelIps(double vLeft_ips, double vRight_ips) {
  double rpmL = ipsToMotorRpm(vLeft_ips);
  double rpmR = ipsToMotorRpm(vRight_ips);

  const double maxRpm = maxMotorRpmLimit();

  // Preserve curvature: if either side saturates, scale both equally.
  const double maxAbs = std::max(std::abs(rpmL), std::abs(rpmR));
  if (maxAbs > maxRpm && maxAbs > 1e-9) {
    const double scale = maxRpm / maxAbs;
    rpmL *= scale;
    rpmR *= scale;
  }

  rpmL = clamp(rpmL, -maxRpm, +maxRpm);
  rpmR = clamp(rpmR, -maxRpm, +maxRpm);

  leftMotors_.moveVelocity(static_cast<std::int16_t>(std::lround(rpmL)));
  rightMotors_.moveVelocity(static_cast<std::int16_t>(std::lround(rpmR)));
}

void TankDrive::setChassisVelocity(const aon::hpp::Command &cmd) {
  // Differential kinematics (omega CCW+):
  //   vL = vx - omega*(W/2)
  //   vR = vx + omega*(W/2)
  const double halfW = 0.5 * kin_.track_width_in;
  const double vL_ips = cmd.vx_ips - (cmd.omega_rps * halfW);
  const double vR_ips = cmd.vx_ips + (cmd.omega_rps * halfW);

  setWheelVelIps(vL_ips, vR_ips);
}

void TankDrive::stop() {
  leftMotors_.moveVelocity(0);
  rightMotors_.moveVelocity(0);
}

void TankDrive::setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode) {
  leftMotors_.setBrakeMode(brakeMode);
  rightMotors_.setBrakeMode(brakeMode);
}

void TankDrive::setGearset(okapi::AbstractMotor::gearset gearset) {
  leftMotors_.setGearing(gearset);
  rightMotors_.setGearing(gearset);
}

void TankDrive::setEncoderUnits(okapi::AbstractMotor::encoderUnits units) {
  leftMotors_.setEncoderUnits(units);
  rightMotors_.setEncoderUnits(units);
  leftMotors_.tarePosition();
  rightMotors_.tarePosition();
}

// ==============================
// TankDriveFeeder (SensorFeeder-style)
// ==============================

void TankDriveFeeder::applyDriveDefaults(TankDrive &dt, const aon::hpp::Config &hppCfg) const {
  // Static motor config
  dt.setBrakeMode(cfg_.brake);
  dt.setGearset(cfg_.gear);
  dt.setEncoderUnits(cfg_.enc);

  // Geometry (lives in cfg_ defaults; main.cpp never touches it)
  dt.setKinematics(cfg_.kin);

  // Single source of truth: bind the same mp_cfg that HPP uses
  dt.bindMotionProfileConfig(hppCfg.mp_cfg);
}

}  // namespace aon
