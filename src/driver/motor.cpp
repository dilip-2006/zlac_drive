#include "zlac_drive/driver/motor.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace zlac_drive::driver
{

namespace
{

constexpr double kPi =
  3.14159265358979323846;

}  // namespace


Motor::Motor(const MotorConfig & config)
: config_(config)
{
  if (config_.gear_ratio <= 0.0) {
    throw std::invalid_argument(
      "Motor gear ratio must be greater than zero");
  }

  if (
    config_.encoder_counts_verified &&
    config_.encoder_counts_per_revolution == 0)
  {
    throw std::invalid_argument(
      "Verified encoder counts per revolution must be greater than zero");
  }
}

double Motor::encoder_counts_to_radians(
  const int64_t encoder_counts) const
{
  if (!config_.encoder_counts_verified) {
    throw std::runtime_error(
      "Encoder counts per revolution are not verified");
  }

  if (config_.encoder_counts_per_revolution == 0) {
    throw std::runtime_error(
      "Encoder counts per revolution must be greater than zero");
  }

  const double counts_per_revolution =
    static_cast<double>(
      config_.encoder_counts_per_revolution);

  return
    static_cast<double>(encoder_counts) *
    (2.0 * kPi / counts_per_revolution);
}

int64_t Motor::radians_to_encoder_counts(
  const double radians) const
{
  if (!config_.encoder_counts_verified) {
    throw std::runtime_error(
      "Encoder counts per revolution are not verified");
  }

  if (config_.encoder_counts_per_revolution == 0) {
    throw std::runtime_error(
      "Encoder counts per revolution must be greater than zero");
  }

  const double counts_per_revolution =
    static_cast<double>(
      config_.encoder_counts_per_revolution);

  const double counts =
    radians *
    (counts_per_revolution /
    (2.0 * kPi));

  return static_cast<int64_t>(
    std::llround(counts));
}

double Motor::rpm_to_rad_per_sec(
  const double rpm)
{
  return rpm *
    (2.0 * kPi / 60.0);
}

double Motor::rad_per_sec_to_rpm(
  const double radians_per_second)
{
  return radians_per_second *
    (60.0 / (2.0 * kPi));
}

}  // namespace zlac_drive::driver