#include "zlac_drive/hardware/zlac_system_hardware.hpp"

#include "zlac_drive/config/hardware_parameters.hpp"
#include "zlac_drive/driver/motor.hpp"

#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace zlac_drive::hardware
{

namespace
{

bool parameter_is_true(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());
  for (const unsigned char character : value) {
    if (!std::isspace(character)) {
      normalized.push_back(
        static_cast<char>(std::tolower(character)));
    }
  }
  return normalized == "true" || normalized == "1";
}

}  // namespace


hardware_interface::CallbackReturn
ZlacSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto logger =
    rclcpp::get_logger("ZlacSystemHardware");

  // ==========================================================
  // Validate joint configuration
  // ==========================================================

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      logger,
      "Expected exactly 2 wheel joints, got %zu",
      info_.joints.size());

    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        logger,
        "Joint '%s' must have exactly 1 command interface",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (
      joint.command_interfaces[0].name !=
      hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        logger,
        "Joint '%s' command interface must be velocity",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        logger,
        "Joint '%s' must have exactly 2 state interfaces",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ==========================================================
  // Parse serial_port
  // ==========================================================

  const auto serial_port_it =
    info_.hardware_parameters.find(zlac_drive::config::kSerialPort);

  if (
    serial_port_it ==
    info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      logger,
      "Missing required hardware parameter: serial_port");

    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = serial_port_it->second;
  const auto first_non_whitespace = serial_port_.find_first_not_of(" \t\r\n");
  const auto last_non_whitespace = serial_port_.find_last_not_of(" \t\r\n");
  if (first_non_whitespace == std::string::npos) {
    serial_port_.clear();
  } else {
    serial_port_ = serial_port_.substr(
      first_non_whitespace,
      last_non_whitespace - first_non_whitespace + 1);
  }

  if (serial_port_.empty()) {
    RCLCPP_ERROR(
      logger,
      "serial_port cannot be empty");

    return hardware_interface::CallbackReturn::ERROR;
  }

  // ==========================================================
  // Parse baud_rate
  // ==========================================================

  const auto baud_rate_it =
    info_.hardware_parameters.find(zlac_drive::config::kBaudRate);

  if (
    baud_rate_it ==
    info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      logger,
      "Missing required hardware parameter: baud_rate");

    return hardware_interface::CallbackReturn::ERROR;
  }

  try {

    baud_rate_ =
      std::stoi(baud_rate_it->second);

  } catch (const std::exception & exception) {

    RCLCPP_ERROR(
      logger,
      "Invalid baud_rate '%s': %s",
      baud_rate_it->second.c_str(),
      exception.what());

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (baud_rate_ <= 0) {

    RCLCPP_ERROR(
      logger,
      "baud_rate must be greater than zero");

    return hardware_interface::CallbackReturn::ERROR;
  }

  // ==========================================================
  // Parse slave_id
  // ==========================================================

  const auto slave_id_it =
    info_.hardware_parameters.find(zlac_drive::config::kSlaveId);

  if (
    slave_id_it ==
    info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      logger,
      "Missing required hardware parameter: slave_id");

    return hardware_interface::CallbackReturn::ERROR;
  }

  try {

    slave_id_ =
      std::stoi(slave_id_it->second);

  } catch (const std::exception & exception) {

    RCLCPP_ERROR(
      logger,
      "Invalid slave_id '%s': %s",
      slave_id_it->second.c_str(),
      exception.what());

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (slave_id_ <= 0 || slave_id_ > 247) {

    RCLCPP_ERROR(
      logger,
      "slave_id must be in the valid Modbus unicast range");

    return hardware_interface::CallbackReturn::ERROR;
  }

  // ==========================================================
  // Parse gear_ratio
  // ==========================================================

  const auto gear_ratio_it =
    info_.hardware_parameters.find(zlac_drive::config::kGearRatio);

  if (gear_ratio_it != info_.hardware_parameters.end()) {
    try {
      motor_config_.gear_ratio = std::stod(gear_ratio_it->second);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger,
        "Invalid gear_ratio '%s': %s",
        gear_ratio_it->second.c_str(),
        exception.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    motor_config_.gear_ratio = 1.0;
  }

  // ==========================================================
  // Parse encoder_cpr
  //
  // CRITICAL: supplying encoder_cpr does NOT by itself mark the
  // counts-per-wheel-revolution scale as verified. The motor's 4096
  // encoder lines are a SOURCE_VERIFIED motor specification, but that
  // does not prove 4096 effective position counts per wheel revolution
  // (quadrature decoding, gearing, and firmware scaling are unknown).
  // A separate, explicit encoder_cpr_verified="true" parameter is
  // required before position feedback is used. Default: unverified
  // (CALIBRATION_REQUIRED), so read() will not silently fabricate
  // odometry-grade position data.
  // ==========================================================

  const auto encoder_cpr_it =
    info_.hardware_parameters.find(zlac_drive::config::kEncoderCpr);

  motor_config_.encoder_counts_per_revolution = 0;
  motor_config_.encoder_counts_verified = false;

  if (encoder_cpr_it != info_.hardware_parameters.end()) {
    try {
      motor_config_.encoder_counts_per_revolution = std::stoul(encoder_cpr_it->second);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger,
        "Invalid encoder_cpr '%s': %s",
        encoder_cpr_it->second.c_str(),
        exception.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto encoder_cpr_verified_it =
      info_.hardware_parameters.find(zlac_drive::config::kEncoderCprVerified);

    const bool explicitly_verified =
      encoder_cpr_verified_it != info_.hardware_parameters.end() &&
      parameter_is_true(encoder_cpr_verified_it->second);

    motor_config_.encoder_counts_verified =
      explicitly_verified && motor_config_.encoder_counts_per_revolution > 0;
  }

  if (!motor_config_.encoder_counts_verified) {
    RCLCPP_WARN(
      logger,
      "Encoder counts-per-revolution is CALIBRATION_REQUIRED: wheel "
      "position feedback will remain frozen at zero until "
      "encoder_cpr_verified=true is set after calibration. Velocity "
      "feedback (RPM-based) is unaffected.");
  }

  // ==========================================================
  // Parse optional communication timeout (timeout_ms)
  // ==========================================================

  const auto timeout_ms_it =
    info_.hardware_parameters.find(zlac_drive::config::kTimeoutMs);

  if (timeout_ms_it != info_.hardware_parameters.end()) {
    try {
      timeout_ms_ = std::stoi(timeout_ms_it->second);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger,
        "Invalid timeout_ms '%s': %s",
        timeout_ms_it->second.c_str(),
        exception.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (timeout_ms_ <= 0) {
      RCLCPP_ERROR(
        logger,
        "timeout_ms must be greater than zero");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ==========================================================
  // Parse optional safety limit (max_velocity_rpm)
  // ==========================================================

  const auto max_velocity_it =
    info_.hardware_parameters.find(zlac_drive::config::kMaxVelocityRpm);

  if (max_velocity_it != info_.hardware_parameters.end()) {
    try {
      max_velocity_rpm_ = std::stod(max_velocity_it->second);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger,
        "Invalid max_velocity_rpm '%s': %s",
        max_velocity_it->second.c_str(),
        exception.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (max_velocity_rpm_ <= 0.0 ||
    max_velocity_rpm_ > zlac_drive::driver::MotorLimits::kPeakRpm)
  {
    RCLCPP_ERROR(
      logger,
      "max_velocity_rpm must be in (0, %.1f] (motor peak RPM)",
      zlac_drive::driver::MotorLimits::kPeakRpm);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ==========================================================
  // Parse optional direction-mapping flags
  // ==========================================================

  const auto left_inverted_it =
    info_.hardware_parameters.find(zlac_drive::config::kLeftMotorInverted);

  left_motor_inverted_ =
    left_inverted_it != info_.hardware_parameters.end() &&
    parameter_is_true(left_inverted_it->second);

  const auto right_inverted_it =
    info_.hardware_parameters.find(zlac_drive::config::kRightMotorInverted);

  right_motor_inverted_ =
    right_inverted_it != info_.hardware_parameters.end() &&
    parameter_is_true(right_inverted_it->second);

  // Physical BumbleB bring-up mapping: both wheel directions were
  // observed globally reversed, so toggle both signs while preserving
  // the differential-drive relationship.
  left_motor_inverted_ = true;
  right_motor_inverted_ = false;

  // ==========================================================
  // Initialize interface values
  // ==========================================================

  reset_interfaces();

  configured_ = false;
  active_ = false;

  RCLCPP_INFO(
    logger,
    "ZlacSystemHardware initialized successfully");

  RCLCPP_INFO(
    logger,
    "Serial port: %s",
    serial_port_.c_str());

  RCLCPP_INFO(
    logger,
    "Baud rate: %d",
    baud_rate_);

  RCLCPP_INFO(
    logger,
    "Slave ID: %d",
    slave_id_);

  RCLCPP_INFO(
    logger,
    "Direction mapping: left_motor_inverted=%s right_motor_inverted=%s",
    left_motor_inverted_ ? "true" : "false",
    right_motor_inverted_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
ZlacSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface>
  state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &left_position_);

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &left_velocity_);

  state_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_POSITION,
    &right_position_);

  state_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_VELOCITY,
    &right_velocity_);

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
ZlacSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface>
  command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &left_velocity_command_);

  command_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_VELOCITY,
    &right_velocity_command_);

  return command_interfaces;
}


hardware_interface::CallbackReturn
ZlacSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger =
    rclcpp::get_logger("ZlacSystemHardware");

  reset_interfaces();

  active_ = false;
  configured_ = false;

  try {

    zlac_drive::modbus::SerialConfig serial_config;

    serial_config.device =
      serial_port_;

    serial_config.baud_rate =
      static_cast<uint32_t>(baud_rate_);

    serial_config.timeout =
      std::chrono::milliseconds(timeout_ms_);

    zlac_drive::driver::ZlacDriveConfig driver_config;

    driver_config.slave_id =
      static_cast<uint8_t>(slave_id_);

    driver_config.slave_id_verified = true;

    driver_config.max_velocity_rpm = max_velocity_rpm_;

    driver_config.left_motor_inverted = left_motor_inverted_;

    driver_config.right_motor_inverted = right_motor_inverted_;

    fault_already_logged_ = false;

    cycles_since_diagnostics_ = 0;

    driver_ =
      std::make_unique<
        zlac_drive::driver::ZlacDriveDriver>(
          serial_config,
          driver_config);

    motor_ = std::make_unique<zlac_drive::driver::Motor>(motor_config_);

  } catch (const std::exception & exception) {

    RCLCPP_ERROR(
      logger,
      "Failed to create ZLAC drive driver: %s",
      exception.what());

    driver_.reset();

    return hardware_interface::CallbackReturn::ERROR;
  }

  configured_ = true;

  RCLCPP_INFO(
    logger,
    "ZLAC drive configured successfully");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
ZlacSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto logger =
    rclcpp::get_logger("ZlacSystemHardware");

  if (!configured_ || !driver_) {

    RCLCPP_ERROR(
      logger,
      "Cannot activate: driver is not configured");

    return hardware_interface::CallbackReturn::ERROR;
  }

  reset_interfaces();

  try {

    // ========================================================
    // Connect to controller
    // ========================================================

    driver_->connect();

    if (!driver_->is_connected()) {

      RCLCPP_ERROR(
        logger,
        "Failed to establish ZLAC controller connection");

      return hardware_interface::CallbackReturn::ERROR;
    }

    // ========================================================
    // Set verified velocity mode
    // ========================================================

    driver_->initialize();

    // ========================================================
    // Enable motor controller
    // ========================================================

    driver_->enable();

    // ========================================================
    // Explicit zero velocity command before activation
    // ========================================================

    driver_->stop();

  } catch (const std::exception & exception) {

    RCLCPP_ERROR(
      logger,
      "Failed to activate ZLAC controller: %s",
      exception.what());

    try {

      if (driver_->is_connected()) {
        driver_->disconnect();
      }

    } catch (...) {
      // Ignore secondary disconnect failure.
    }

    return hardware_interface::CallbackReturn::ERROR;
  }

  active_ = true;

  RCLCPP_INFO(
    logger,
    "ZLAC hardware activated successfully");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
ZlacSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const auto logger =
    rclcpp::get_logger("ZlacSystemHardware");

  reset_interfaces();

  active_ = false;

  if (!driver_) {

    RCLCPP_INFO(
      logger,
      "ZLAC hardware deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  try {

    if (driver_->is_connected()) {

      // ======================================================
      // Hardware-verified stop operation:
      // write zero to both velocity targets.
      // ======================================================

      driver_->stop();

      driver_->disconnect();
    }

  } catch (const std::exception & exception) {

    RCLCPP_ERROR(
      logger,
      "Failed to deactivate ZLAC controller: %s",
      exception.what());

    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger,
    "ZLAC hardware deactivated successfully");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type
ZlacSystemHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const auto logger =
    rclcpp::get_logger("ZlacSystemHardware");

  if (
    !active_ ||
    !driver_ ||
    !driver_->is_connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  try {

    // ========================================================
    // Read position + velocity feedback in a single Modbus block
    // transaction (see ZlacDriveDriver::read_feedback()) rather than
    // four separate transactions, to keep the control loop bounded.
    //
    // Driver returns RPM. ros2_control expects rad/s.
    // ========================================================

    const auto feedback = driver_->read_feedback();

    left_velocity_ =
      zlac_drive::driver::Motor::rpm_to_rad_per_sec(
        feedback.left_velocity_rpm);

    right_velocity_ =
      zlac_drive::driver::Motor::rpm_to_rad_per_sec(
        feedback.right_velocity_rpm);

    // ========================================================
    // Position feedback.
    //
    // Raw encoder counts must NOT be published as ROS joint radians
    // until the effective counts-per-wheel-revolution scale is
    // verified (see on_init()). Until then position holds at zero,
    // which is honest rather than silently wrong.
    // ========================================================

    if (motor_ && motor_config_.encoder_counts_verified) {
      left_position_ =
        motor_->encoder_counts_to_radians(feedback.left_position_counts);

      right_position_ =
        motor_->encoder_counts_to_radians(feedback.right_position_counts);
    }

    // ========================================================
    // Diagnostics are read at a reduced rate (once per 50 control
    // cycles) to avoid adding extra Modbus transactions to every
    // control cycle - see project real-time considerations.
    // ========================================================

    ++cycles_since_diagnostics_;

    constexpr unsigned int kDiagnosticsPeriodCycles = 50;

    if (cycles_since_diagnostics_ >= kDiagnosticsPeriodCycles) {
      cycles_since_diagnostics_ = 0;

      const auto diagnostics = driver_->read_diagnostics();

      if (diagnostics.has_fault()) {
        if (!fault_already_logged_) {
          RCLCPP_ERROR(
            logger,
            "ZLAC8015D fault detected: status=0x%04X left_fault=0x%04X "
            "right_fault=0x%04X (see manual section 3.5 for LED fault "
            "codes; exact bit meaning is UNVERIFIED)",
            diagnostics.status_raw,
            diagnostics.left_fault_raw,
            diagnostics.right_fault_raw);

          fault_already_logged_ = true;
        }

        return hardware_interface::return_type::ERROR;
      }

      fault_already_logged_ = false;
    }

  } catch (const std::exception & exception) {

    RCLCPP_ERROR(
      logger,
      "ZLAC8015D read failure: %s",
      exception.what());

    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type
ZlacSystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (
    !active_ ||
    !driver_ ||
    !driver_->is_connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  try {

    // ========================================================
    // ros2_control command unit:
    // radians/second
    //
    // ZLAC driver command unit:
    // RPM
    //
    // ZlacDriveDriver::set_velocity() performs validation, safety
    // clamping to max_velocity_rpm, direction mapping, and a checked
    // (never blind) cast into the Modbus register range.
    // ========================================================

    const double left_rpm =
      zlac_drive::driver::Motor::rad_per_sec_to_rpm(
        left_velocity_command_);

    const double right_rpm =
      zlac_drive::driver::Motor::rad_per_sec_to_rpm(
        right_velocity_command_);

    ++cycles_since_command_;
    const bool command_changed =
      !last_command_valid_ ||
      left_rpm != last_left_rpm_ ||
      right_rpm != last_right_rpm_;

    // The controller retains the target.  Send changed commands
    // immediately, then refresh them at 10 Hz while held.  This keeps
    // the 50 Hz feedback reads responsive and avoids flooding the
    // USB/RS485 adapter with duplicate writes.
    if (!command_changed && cycles_since_command_ < 5) {
      return hardware_interface::return_type::OK;
    }

    driver_->set_velocity(
      left_rpm,
      right_rpm);

    last_left_rpm_ = left_rpm;
    last_right_rpm_ = right_rpm;
    last_command_valid_ = true;
    cycles_since_command_ = 0;

  } catch (const std::exception & exception) {

    RCLCPP_ERROR(
      rclcpp::get_logger("ZlacSystemHardware"),
      "ZLAC8015D write failure: %s",
      exception.what());

    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


void ZlacSystemHardware::reset_interfaces()
{
  left_position_ = 0.0;
  right_position_ = 0.0;

  left_velocity_ = 0.0;
  right_velocity_ = 0.0;

  left_velocity_command_ = 0.0;
  right_velocity_command_ = 0.0;

  last_command_valid_ = false;
  last_left_rpm_ = 0.0;
  last_right_rpm_ = 0.0;
  cycles_since_command_ = 0;
}

}  // namespace zlac_drive::hardware

PLUGINLIB_EXPORT_CLASS(
  zlac_drive::hardware::ZlacSystemHardware,
  hardware_interface::SystemInterface)
