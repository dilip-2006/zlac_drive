#ifndef ZLAC_DRIVE__CONFIG__HARDWARE_PARAMETERS_HPP_
#define ZLAC_DRIVE__CONFIG__HARDWARE_PARAMETERS_HPP_

namespace zlac_drive::config
{

// Required parameters.
inline constexpr const char * kSerialPort = "serial_port";
inline constexpr const char * kBaudRate = "baud_rate";
inline constexpr const char * kSlaveId = "slave_id";

// Optional motor/encoder conversion parameters.
inline constexpr const char * kEncoderCpr = "encoder_cpr";

/**
 * Explicit, separate flag that must be set to "true" for encoder_cpr to
 * be treated as a verified counts-per-wheel-revolution scale. Supplying
 * encoder_cpr alone does NOT imply verification - see
 * MotorConfig::encoder_counts_verified and the CRITICAL ENCODER RULE in
 * project documentation. Defaults to false (CALIBRATION_REQUIRED).
 */
inline constexpr const char * kEncoderCprVerified = "encoder_cpr_verified";

inline constexpr const char * kGearRatio = "gear_ratio";

// Optional communication parameters.
inline constexpr const char * kTimeoutMs = "timeout_ms";

// Optional safety parameters.

/**
 * Maximum commandable motor speed, RPM. Must never exceed
 * zlac_drive::driver::MotorLimits::kPeakRpm. Defaults to the peak RPM
 * from the motor specification if not supplied.
 */
inline constexpr const char * kMaxVelocityRpm = "max_velocity_rpm";

// Optional direction-mapping parameters (see zlac_drive README for how
// to verify physical wheel direction on real hardware).
inline constexpr const char * kLeftMotorInverted = "left_motor_inverted";
inline constexpr const char * kRightMotorInverted = "right_motor_inverted";

}  // namespace zlac_drive::config

#endif
