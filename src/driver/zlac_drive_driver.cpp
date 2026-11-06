#include "zlac_drive/driver/zlac_drive_driver.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "zlac_drive/modbus/modbus_exception.hpp"
#include "zlac_drive/registers.hpp"

namespace zlac_drive::driver
{

namespace
{

int32_t registers_to_signed_int32(
  const uint16_t high_word,
  const uint16_t low_word)
{
  const uint32_t combined =
    (static_cast<uint32_t>(high_word) << 16) |
    static_cast<uint32_t>(low_word);

  return static_cast<int32_t>(combined);
}

int16_t register_to_signed_int16(
  const uint16_t value)
{
  return static_cast<int16_t>(value);
}

uint16_t signed_int16_to_register(
  const int16_t value)
{
  return static_cast<uint16_t>(value);
}

}  // namespace


ZlacDriveDriver::ZlacDriveDriver(
  const modbus::SerialConfig & serial_config,
  const ZlacDriveConfig & driver_config)
: modbus_client_(serial_config),
  config_(driver_config)
{
  if (config_.max_velocity_rpm <= 0.0 ||
    config_.max_velocity_rpm > MotorLimits::kPeakRpm)
  {
    throw std::invalid_argument(
      "max_velocity_rpm must be in (0, MotorLimits::kPeakRpm]");
  }
}


void ZlacDriveDriver::connect()
{
  if (is_connected()) {
    return;
  }

  set_state(DriverState::CONNECTING);

  try {
    modbus_client_.connect();
  } catch (const modbus::ModbusException &) {
    set_state(DriverState::COMMUNICATION_ERROR);
    throw;
  }

  diagnostics_.communication_ok = true;

  set_state(DriverState::CONNECTED);
}


void ZlacDriveDriver::disconnect()
{
  if (modbus_client_.is_connected()) {
    modbus_client_.disconnect();
  }

  diagnostics_.communication_ok = false;

  set_state(DriverState::DISCONNECTED);
}


bool ZlacDriveDriver::is_connected() const noexcept
{
  return modbus_client_.is_connected();
}


DriverState ZlacDriveDriver::state() const noexcept
{
  return state_;
}


void ZlacDriveDriver::initialize()
{
  if (!is_connected()) {
    throw std::runtime_error(
      "Cannot initialize ZLAC8015D: not connected");
  }

  if (!config_.slave_id_verified) {
    throw std::runtime_error(
      "Cannot initialize ZLAC8015D: Modbus slave ID is not verified");
  }

  if (state_ == DriverState::INITIALIZED || state_ == DriverState::ENABLED) {
    return;
  }

  try {
    modbus_client_.write_single_register(
      config_.slave_id,
      zlac_drive::registers::control::CONTROL_MODE,
      zlac_drive::registers::control::VELOCITY_MODE);
  } catch (const modbus::ModbusException &) {
    set_state(DriverState::COMMUNICATION_ERROR);
    throw;
  }

  set_state(DriverState::INITIALIZED);
}


void ZlacDriveDriver::enable()
{
  if (!is_connected()) {
    throw std::runtime_error(
      "Cannot enable ZLAC8015D: not connected");
  }

  if (state_ != DriverState::INITIALIZED && state_ != DriverState::ENABLED) {
    throw std::runtime_error(
      "Cannot enable ZLAC8015D: driver is not initialized");
  }

  if (state_ == DriverState::ENABLED) {
    return;
  }

  try {
    modbus_client_.write_single_register(
      config_.slave_id,
      zlac_drive::registers::control::CONTROL_COMMAND,
      zlac_drive::registers::control::ENABLE_COMMAND);
  } catch (const modbus::ModbusException &) {
    set_state(DriverState::COMMUNICATION_ERROR);
    throw;
  }

  set_state(DriverState::ENABLED);
}


void ZlacDriveDriver::disable()
{
  if (!is_connected()) {
    throw std::runtime_error(
      "Cannot disable ZLAC8015D: not connected");
  }

  /*
   * No verified controller disable/quick-stop command exists (see
   * registers.hpp). Commanding zero velocity is the verified safe-stop
   * behavior; this is NOT equivalent to a servo/torque disable.
   */

  stop();

  if (state_ == DriverState::ENABLED) {
    set_state(DriverState::INITIALIZED);
  }
}


double ZlacDriveDriver::clamp_velocity_rpm(
  const double velocity_rpm,
  const double max_rpm)
{
  if (!std::isfinite(velocity_rpm)) {
    return 0.0;
  }

  if (velocity_rpm > max_rpm) {
    return max_rpm;
  }

  if (velocity_rpm < -max_rpm) {
    return -max_rpm;
  }

  return velocity_rpm;
}


int16_t ZlacDriveDriver::to_velocity_register_value(
  const double clamped_velocity_rpm)
{
  const double rounded =
    std::round(clamped_velocity_rpm);

  constexpr double kInt16Min =
    static_cast<double>(std::numeric_limits<int16_t>::min());

  constexpr double kInt16Max =
    static_cast<double>(std::numeric_limits<int16_t>::max());

  if (rounded < kInt16Min || rounded > kInt16Max) {
    throw std::out_of_range(
      "Clamped velocity command does not fit in a signed 16-bit "
      "Modbus register");
  }

  return static_cast<int16_t>(rounded);
}


void ZlacDriveDriver::set_velocity(
  const double left_velocity_rpm,
  const double right_velocity_rpm)
{
  if (!is_connected()) {
    throw std::runtime_error(
      "Cannot command velocity: ZLAC8015D is not connected");
  }

  if (state_ != DriverState::ENABLED) {
    throw std::runtime_error(
      "Cannot command velocity: driver is not enabled");
  }

  // 1. Validate (non-finite treated as zero) and 2. clamp to the
  // configured safe range.
  const double left_clamped =
    clamp_velocity_rpm(left_velocity_rpm, config_.max_velocity_rpm);

  const double right_clamped =
    clamp_velocity_rpm(right_velocity_rpm, config_.max_velocity_rpm);

  // 3. Apply explicit, configurable direction mapping.
  const double left_signed =
    config_.left_motor_inverted ? -left_clamped : left_clamped;

  const double right_signed =
    config_.right_motor_inverted ? -right_clamped : right_clamped;

  // 4/5. Intentional round and range-check into int16_t (never an
  // unchecked cast).
  const int16_t left_rpm =
    to_velocity_register_value(left_signed);

  const int16_t right_rpm =
    to_velocity_register_value(right_signed);

  const std::vector<uint16_t> commands{
    signed_int16_to_register(left_rpm),
    signed_int16_to_register(right_rpm)
  };

  try {
    modbus_client_.write_multiple_registers(
      config_.slave_id,
      zlac_drive::registers::control::LEFT_TARGET_VELOCITY,
      commands);
  } catch (const modbus::ModbusException &) {
    set_state(DriverState::COMMUNICATION_ERROR);
    throw;
  }
}


RawFeedback ZlacDriveDriver::read_feedback()
{
  if (!is_connected()) {
    throw std::runtime_error(
      "Cannot read feedback: ZLAC8015D is not connected");
  }

  std::vector<uint16_t> response;

  try {
    response = modbus_client_.read_holding_registers(
      config_.slave_id,
      zlac_drive::registers::feedback::FEEDBACK_BLOCK_START,
      zlac_drive::registers::feedback::FEEDBACK_BLOCK_COUNT);
  } catch (const modbus::ModbusException &) {
    set_state(DriverState::COMMUNICATION_ERROR);
    throw;
  }

  if (response.size() !=
    zlac_drive::registers::feedback::FEEDBACK_BLOCK_COUNT)
  {
    set_state(DriverState::COMMUNICATION_ERROR);
    throw std::runtime_error(
      "Unexpected response size while reading feedback block");
  }

  // Block layout (registers.hpp feedback::FEEDBACK_BLOCK_START..+6):
  //   [0] left position high, [1] left position low,
  //   [2] right position high, [3] right position low,
  //   [4] left velocity,       [5] right velocity.
  RawFeedback feedback;

  const int32_t left_position_raw =
    registers_to_signed_int32(response[0], response[1]);

  const int32_t right_position_raw =
    registers_to_signed_int32(response[2], response[3]);

  const double left_velocity_raw =
    static_cast<double>(register_to_signed_int16(response[4])) /
    zlac_drive::registers::feedback::VELOCITY_SCALE_RPM;

  const double right_velocity_raw =
    static_cast<double>(register_to_signed_int16(response[5])) /
    zlac_drive::registers::feedback::VELOCITY_SCALE_RPM;

  // Apply the same explicit direction mapping used for commands so
  // feedback sign matches commanded sign.
  feedback.left_position_counts =
    config_.left_motor_inverted ? -left_position_raw : left_position_raw;

  feedback.right_position_counts =
    config_.right_motor_inverted ? -right_position_raw : right_position_raw;

  feedback.left_velocity_rpm =
    config_.left_motor_inverted ? -left_velocity_raw : left_velocity_raw;

  feedback.right_velocity_rpm =
    config_.right_motor_inverted ? -right_velocity_raw : right_velocity_raw;

  return feedback;
}


diagnostics::DriverDiagnostics ZlacDriveDriver::read_diagnostics()
{
  if (!is_connected()) {
    throw std::runtime_error(
      "Cannot read diagnostics: ZLAC8015D is not connected");
  }

  try {
    const auto status_response =
      modbus_client_.read_holding_registers(
        config_.slave_id,
        zlac_drive::registers::status::DRIVER_STATUS,
        1);

    const auto left_fault_response =
      modbus_client_.read_holding_registers(
        config_.slave_id,
        zlac_drive::registers::fault::LEFT_FAULT,
        1);

    const auto right_fault_response =
      modbus_client_.read_holding_registers(
        config_.slave_id,
        zlac_drive::registers::fault::RIGHT_FAULT,
        1);

    diagnostics_.status_raw = status_response.at(0);
    diagnostics_.left_fault_raw = left_fault_response.at(0);
    diagnostics_.right_fault_raw = right_fault_response.at(0);
    diagnostics_.communication_ok = true;
  } catch (const modbus::ModbusException &) {
    diagnostics_.communication_ok = false;
    set_state(DriverState::COMMUNICATION_ERROR);
    throw;
  }

  if (diagnostics_.has_fault() && state_ != DriverState::FAULT) {
    set_state(DriverState::FAULT);
  }

  diagnostics_.state = state_;

  return diagnostics_;
}


const diagnostics::DriverDiagnostics & ZlacDriveDriver::last_diagnostics()
const noexcept
{
  return diagnostics_;
}


void ZlacDriveDriver::stop()
{
  if (!is_connected()) {
    return;
  }

  const std::vector<uint16_t> commands{
    signed_int16_to_register(0),
    signed_int16_to_register(0)
  };

  try {
    modbus_client_.write_multiple_registers(
      config_.slave_id,
      zlac_drive::registers::control::LEFT_TARGET_VELOCITY,
      commands);
  } catch (const modbus::ModbusException &) {
    set_state(DriverState::COMMUNICATION_ERROR);
    throw;
  }
}


void ZlacDriveDriver::set_state(const DriverState new_state)
{
  state_ = new_state;
  diagnostics_.state = new_state;
}

}  // namespace zlac_drive::driver
