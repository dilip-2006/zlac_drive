#ifndef ZLAC_DRIVE__HARDWARE__ZLAC_SYSTEM_HARDWARE_HPP_
#define ZLAC_DRIVE__HARDWARE__ZLAC_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "zlac_drive/driver/zlac_drive_driver.hpp"

namespace zlac_drive::hardware
{

/**
 * @brief ros2_control hardware interface for the ZLAC8015D.
 *
 * This class is responsible for ROS 2 hardware integration.
 *
 * Responsibilities:
 *
 * - Parse ros2_control hardware parameters.
 * - Export wheel position and velocity state interfaces.
 * - Export wheel velocity command interfaces.
 * - Manage the lifecycle of ZlacDriveDriver.
 * - Convert between ros2_control state/command interfaces and the
 *   reusable ZLAC driver layer.
 *
 * The ZLAC8015D Modbus register protocol belongs in ZlacDriveDriver,
 * not in this class.
 */
class ZlacSystemHardware
  : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ZlacSystemHardware)

  /**
   * @brief Initialize hardware configuration from ros2_control URDF.
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Export wheel position and velocity state interfaces.
   */
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  /**
   * @brief Export wheel velocity command interfaces.
   */
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /**
   * @brief Configure the ZLAC driver.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Connect, initialize, and activate the motor controller.
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Stop and deactivate the motor controller.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read wheel state from the motor controller.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Send wheel velocity commands to the motor controller.
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  /**
   * @brief Reset exported joint state and commands to zero.
   */
  void reset_interfaces();

  // ============================================================
  // Hardware communication configuration
  // ============================================================

  std::string serial_port_;

  int baud_rate_{115200};

  int slave_id_{1};

  int timeout_ms_{100};

  double max_velocity_rpm_{
    zlac_drive::driver::MotorLimits::kPeakRpm};

  bool left_motor_inverted_{false};

  bool right_motor_inverted_{false};

  // ============================================================
  // Reusable ZLAC driver
  // ============================================================

  std::unique_ptr<zlac_drive::driver::ZlacDriveDriver> driver_;

  zlac_drive::driver::MotorConfig motor_config_;
  std::unique_ptr<zlac_drive::driver::Motor> motor_;

  // ============================================================
  // ros2_control state interfaces
  // ============================================================

  double left_position_{0.0};

  double right_position_{0.0};

  double left_velocity_{0.0};

  double right_velocity_{0.0};

  // ============================================================
  // ros2_control command interfaces
  // ============================================================

  double left_velocity_command_{0.0};

  double right_velocity_command_{0.0};

  // The ZLAC controller holds its target velocity.  Avoid sending the
  // same Modbus write on every ros2_control cycle; repeated FC 0x10
  // traffic can overrun some USB/RS485 adapters.  Commands are sent
  // immediately when they change and periodically while held.
  bool last_command_valid_{false};
  double last_left_rpm_{0.0};
  double last_right_rpm_{0.0};
  unsigned int cycles_since_command_{0};

  // ============================================================
  // Hardware lifecycle state
  // ============================================================

  bool configured_{false};

  bool active_{false};

  /**
   * Edge-triggered fault-logging state: true once a fault has already
   * been logged, so read() does not spam the log every control cycle.
   */
  bool fault_already_logged_{false};

  /**
   * Diagnostics are read at a fraction of the control rate to keep the
   * high-frequency read()/write() loop bounded (see project real-time
   * considerations). Counts control cycles since the last diagnostics
   * read.
   */
  unsigned int cycles_since_diagnostics_{0};
};

}  // namespace zlac_drive::hardware

#endif  // ZLAC_DRIVE__HARDWARE__ZLAC_SYSTEM_HARDWARE_HPP_
