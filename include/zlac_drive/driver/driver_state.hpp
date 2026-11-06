#ifndef ZLAC_DRIVE__DRIVER__DRIVER_STATE_HPP_
#define ZLAC_DRIVE__DRIVER__DRIVER_STATE_HPP_

namespace zlac_drive::driver
{

/**
 * @brief Represents the lifecycle state of the ZLAC8015D motor controller driver.
 *
 * This is the single canonical state definition for ZlacDriveDriver.
 * Do not duplicate these states elsewhere.
 *
 * Valid transitions:
 *
 *   DISCONNECTED -> CONNECTING -> CONNECTED -> INITIALIZED -> ENABLED
 *   (any state)  -> COMMUNICATION_ERROR  (Modbus transport failure)
 *   (any state)  -> FAULT                (controller-reported fault)
 *   ENABLED      -> INITIALIZED          (disable(), stops motion first)
 *   (any state)  -> DISCONNECTED         (disconnect())
 */
enum class DriverState
{
  /**
   * Driver is disconnected from the Modbus transport.
   */
  DISCONNECTED,

  /**
   * Modbus connection is being established.
   */
  CONNECTING,

  /**
   * Modbus is connected, but controller is not yet initialized for motion.
   */
  CONNECTED,

  /**
   * Controller is initialized (e.g., velocity mode selected) but motors are disabled.
   */
  INITIALIZED,

  /**
   * Motors are enabled and actively responding to commands.
   */
  ENABLED,

  /**
   * A fault has been detected on the controller (see driver_diagnostics.hpp).
   */
  FAULT,

  /**
   * A Modbus transport failure occurred (timeout, CRC error, malformed
   * response). Distinct from FAULT, which is a controller-reported
   * condition read over a working transport.
   */
  COMMUNICATION_ERROR
};

/**
 * @brief Human-readable name for a DriverState, for logging/diagnostics.
 */
constexpr const char * to_string(const DriverState state)
{
  switch (state) {
    case DriverState::DISCONNECTED:
      return "DISCONNECTED";
    case DriverState::CONNECTING:
      return "CONNECTING";
    case DriverState::CONNECTED:
      return "CONNECTED";
    case DriverState::INITIALIZED:
      return "INITIALIZED";
    case DriverState::ENABLED:
      return "ENABLED";
    case DriverState::FAULT:
      return "FAULT";
    case DriverState::COMMUNICATION_ERROR:
      return "COMMUNICATION_ERROR";
  }

  return "UNKNOWN";
}

}  // namespace zlac_drive::driver

#endif  // ZLAC_DRIVE__DRIVER__DRIVER_STATE_HPP_
