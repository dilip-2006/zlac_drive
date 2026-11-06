#ifndef ZLAC_DRIVE__DRIVER__ZLAC_DRIVE_DRIVER_HPP_
#define ZLAC_DRIVE__DRIVER__ZLAC_DRIVE_DRIVER_HPP_

#include <cstdint>

#include "zlac_drive/diagnostics/driver_diagnostics.hpp"
#include "zlac_drive/driver/driver_state.hpp"
#include "zlac_drive/driver/motor.hpp"
#include "zlac_drive/modbus/modbus_client.hpp"

namespace zlac_drive::driver
{

/**
 * @brief Configuration required by the ZLAC8015D driver layer.
 *
 * This structure contains controller-specific configuration.
 *
 * Robot geometry, wheel separation, wheel radius, and ROS 2 controller
 * parameters belong outside this reusable driver package.
 */
struct ZlacDriveConfig
{
  /**
   * @brief Modbus RTU slave ID.
   */
  uint8_t slave_id{1};

  /**
   * @brief Indicates whether the slave ID has been verified against
   * the physical controller configuration.
   */
  bool slave_id_verified{false};

  /**
   * @brief Maximum commandable |velocity|, RPM.
   *
   * MUST NOT exceed MotorLimits::kPeakRpm. Any velocity command is
   * clamped to this limit before being sent to the controller. This is
   * the real, enforced safety limit referenced in project requirements
   * (not merely documentation).
   */
  double max_velocity_rpm{MotorLimits::kPeakRpm};

  /**
   * @brief Explicit left motor direction mapping.
   *
   * When true, commanded and fed-back left velocity/position sign is
   * inverted before/after the Modbus transaction. This makes the
   * physical wheel-rotation direction configurable without modifying
   * driver logic. Physical direction must be confirmed on real
   * hardware - see zlac_drive/README.md.
   */
  bool left_motor_inverted{false};

  /**
   * @brief Explicit right motor direction mapping. See
   * left_motor_inverted.
   */
  bool right_motor_inverted{false};
};

/**
 * @brief Feedback sampled from the controller in a single Modbus
 * transaction (registers::feedback::FEEDBACK_BLOCK_START,
 * FEEDBACK_BLOCK_COUNT).
 *
 * Positions are raw encoder counts, not radians - see
 * MotorConfig::encoder_counts_verified before converting.
 * Velocities are raw controller units (RPM x
 * registers::feedback::VELOCITY_SCALE_RPM).
 */
struct RawFeedback
{
  int32_t left_position_counts{0};
  int32_t right_position_counts{0};
  double left_velocity_rpm{0.0};
  double right_velocity_rpm{0.0};
};

/**
 * @brief ZLAC8015D Modbus driver abstraction.
 *
 * This class owns controller-specific communication behavior and delegates
 * Modbus RTU transport to modbus::ModbusClient.
 *
 * Architecture:
 *
 *   ros2_control hardware interface
 *              |
 *              v
 *       ZlacDriveDriver
 *              |
 *              v
 *         ModbusClient
 *              |
 *              v
 *          RS-485 / USB
 *
 * Robot-specific geometry and kinematics intentionally do not belong here.
 */
class ZlacDriveDriver
{
public:
  /**
   * @brief Construct the ZLAC8015D driver.
   *
   * @param serial_config Serial/Modbus RTU transport configuration.
   * @param driver_config ZLAC controller configuration.
   */
  ZlacDriveDriver(
    const modbus::SerialConfig & serial_config,
    const ZlacDriveConfig & driver_config);

  ~ZlacDriveDriver() = default;

  ZlacDriveDriver(
    const ZlacDriveDriver &) = delete;

  ZlacDriveDriver & operator=(
    const ZlacDriveDriver &) = delete;

  ZlacDriveDriver(
    ZlacDriveDriver &&) = delete;

  ZlacDriveDriver & operator=(
    ZlacDriveDriver &&) = delete;

  /**
   * @brief Connect to the ZLAC8015D Modbus device.
   *
   * Transitions DISCONNECTED -> CONNECTING -> CONNECTED (or
   * COMMUNICATION_ERROR on failure).
   */
  void connect();

  /**
   * @brief Disconnect from the ZLAC8015D controller.
   *
   * Transitions to DISCONNECTED from any state.
   */
  void disconnect();

  /**
   * @brief Check whether the Modbus transport is connected.
   */
  bool is_connected() const noexcept;

  /**
   * @brief Current driver state machine value.
   */
  DriverState state() const noexcept;

  /**
   * @brief Initialize the motor controller (select velocity mode).
   *
   * Transitions CONNECTED -> INITIALIZED.
   */
  void initialize();

  /**
   * @brief Enable motor output.
   *
   * Transitions INITIALIZED -> ENABLED.
   */
  void enable();

  /**
   * @brief Disable motor output.
   *
   * No verified disable/quick-stop command exists (see registers.hpp),
   * so this commands zero velocity first, then transitions
   * ENABLED -> INITIALIZED. This is a safe-stop, not a servo disable -
   * see project documentation section on safe shutdown.
   */
  void disable();

  /**
   * @brief Command left and right wheel velocities.
   *
   * @param left_velocity_rpm Desired left wheel velocity, RPM.
   * @param right_velocity_rpm Desired right wheel velocity, RPM.
   *
   * Both inputs are validated (rejecting NaN/Inf), clamped to
   * [-max_velocity_rpm, +max_velocity_rpm], have direction mapping
   * applied, and are range-checked before the intentional round to
   * int16_t. Values are never cast to int16_t without this pipeline.
   */
  void set_velocity(
    double left_velocity_rpm,
    double right_velocity_rpm);

  /**
   * @brief Read left and right position/velocity feedback in a single
   * Modbus block-read transaction.
   */
  RawFeedback read_feedback();

  /**
   * @brief Read status/fault registers and update the diagnostics
   * snapshot. Also updates the driver state to FAULT if a fault is
   * newly detected.
   */
  diagnostics::DriverDiagnostics read_diagnostics();

  /**
   * @brief Most recently sampled diagnostics snapshot (does not perform
   * Modbus I/O - call read_diagnostics() to refresh it).
   */
  const diagnostics::DriverDiagnostics & last_diagnostics() const noexcept;

  /**
   * @brief Command both motors to zero velocity.
   *
   * Safe to call regardless of state, as long as connected. Used for
   * command-timeout, shutdown, and pre-activation safety.
   */
  void stop();

  /**
   * @brief Clamp a requested velocity to the closed interval
   * [-max_rpm, +max_rpm], treating non-finite input as zero.
   *
   * Pure function exposed for unit testing of the safety-limit
   * pipeline without requiring hardware.
   */
  static double clamp_velocity_rpm(
    double velocity_rpm,
    double max_rpm);

  /**
   * @brief Intentionally round and range-check a clamped RPM value into
   * the signed 16-bit range accepted by the ZLAC velocity registers.
   *
   * @throws std::out_of_range if the rounded value cannot be
   * represented as int16_t (should not occur for any max_rpm <=
   * MotorLimits::kPeakRpm, but is checked explicitly rather than
   * assumed).
   */
  static int16_t to_velocity_register_value(
    double clamped_velocity_rpm);

private:
  void set_state(DriverState new_state);

  /**
   * @brief Generic Modbus RTU communication layer.
   */
  modbus::ModbusClient modbus_client_;

  /**
   * @brief ZLAC8015D-specific configuration.
   */
  ZlacDriveConfig config_;

  /**
   * @brief Driver lifecycle state machine (see driver_state.hpp).
   */
  DriverState state_{DriverState::DISCONNECTED};

  /**
   * @brief Most recent diagnostics snapshot.
   */
  diagnostics::DriverDiagnostics diagnostics_;
};

}  // namespace zlac_drive::driver

#endif  // ZLAC_DRIVE__DRIVER__ZLAC_DRIVE_DRIVER_HPP_
