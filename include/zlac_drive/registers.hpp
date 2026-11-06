#ifndef ZLAC_DRIVE__REGISTERS_HPP_
#define ZLAC_DRIVE__REGISTERS_HPP_

#include <cstdint>

namespace zlac_drive::registers
{

/**
 * @brief ZLAC8015D Modbus register definitions.
 *
 * Traceability policy (see project verification system):
 *
 *   PHYSICALLY_VERIFIED  - confirmed against a connected ZLAC8015D during
 *                          bring-up (see the note on each register below).
 *   SOURCE_VERIFIED      - explicitly documented in the ZLAC8015D user
 *                          manual supplied with this project.
 *   DERIVED               - mathematically derived from a verified value.
 *   UNVERIFIED            - not confirmed by any of the above. Must not be
 *                          used to command motion.
 *   CALIBRATION_REQUIRED  - requires physical measurement/tuning.
 *
 * The supplied ZLAC8015D user manual (Version 1.02) documents electrical
 * ratings, connector pinouts, LED fault codes and RS-485/CAN transport,
 * but does NOT publish a Modbus register/address table. Every register
 * address below is therefore PHYSICALLY_VERIFIED bring-up knowledge
 * (recorded from direct testing against the controller), not something
 * that can be cross-checked against the manual. Fields that were not
 * exercised during that bring-up (e.g. exact fault bit meanings, an
 * explicit disable/quick-stop command) are explicitly left UNVERIFIED
 * below and must not be invented.
 */

namespace control
{

/**
 * Controller operating mode.
 *
 * Address    : 0x200D
 * Function   : 0x06 Write Single Register
 * Type       : uint16, unsigned
 * Status     : PHYSICALLY_VERIFIED
 * Note       : Writing VELOCITY_MODE (3) successfully selected velocity
 *              mode during bring-up testing.
 */
inline constexpr uint16_t CONTROL_MODE = 0x200D;

inline constexpr uint16_t VELOCITY_MODE = 3;

/**
 * Controller control command.
 *
 * Address    : 0x200E
 * Function   : 0x06 Write Single Register
 * Type       : uint16, unsigned
 * Status     : PHYSICALLY_VERIFIED (enable value only)
 * Note       : Writing ENABLE_COMMAND (0x0008) enabled motor operation
 *              during the velocity-mode bring-up test. No explicit
 *              disable/quick-stop value has been verified - see
 *              ZlacDriveDriver::disable(), which stops via zero velocity
 *              instead of an unverified disable command.
 */
inline constexpr uint16_t CONTROL_COMMAND = 0x200E;

inline constexpr uint16_t ENABLE_COMMAND = 0x0008;

/**
 * Left motor target velocity.
 *
 * Address    : 0x2088
 * Function   : 0x06 / 0x10 (single or block write)
 * Type       : int16, signed
 * Units      : RPM
 * Side       : LEFT
 * Status     : PHYSICALLY_VERIFIED
 */
inline constexpr uint16_t LEFT_TARGET_VELOCITY = 0x2088;

/**
 * Right motor target velocity.
 *
 * Address    : 0x2089
 * Function   : 0x06 / 0x10 (single or block write)
 * Type       : int16, signed
 * Units      : RPM
 * Side       : RIGHT
 * Status     : PHYSICALLY_VERIFIED
 *
 * LEFT_TARGET_VELOCITY and RIGHT_TARGET_VELOCITY are contiguous, allowing
 * both to be written in a single 0x10 block write.
 */
inline constexpr uint16_t RIGHT_TARGET_VELOCITY = 0x2089;

}  // namespace control


namespace feedback
{

/**
 * Feedback block: left position (2 words), right position (2 words),
 * left velocity (1 word), right velocity (1 word).
 *
 * Registers 0x20A7-0x20AC are contiguous and PHYSICALLY_VERIFIED
 * individually. ZlacDriveDriver::read_feedback() reads all six with a
 * single 0x03 block read (FEEDBACK_BLOCK_START..FEEDBACK_BLOCK_COUNT)
 * instead of four separate transactions, per the driver's block-read
 * requirement.
 */
inline constexpr uint16_t FEEDBACK_BLOCK_START = 0x20A7;
inline constexpr uint16_t FEEDBACK_BLOCK_COUNT = 6;

/**
 * Left encoder position.
 *
 * Address    : High word 0x20A7, low word 0x20A8
 * Function   : 0x03 Read Holding Registers
 * Type       : int32, signed (high word first, big-endian word order)
 * Units      : raw encoder counts (NOT radians - see registers.hpp
 *              header note and MotorConfig::encoder_counts_verified)
 * Side       : LEFT
 * Status     : PHYSICALLY_VERIFIED (register presence and word order)
 *              CALIBRATION_REQUIRED (counts-per-wheel-revolution scale)
 *
 * Do not convert raw counts to radians until the effective encoder
 * scale has been calibrated - see zlac_drive/driver/motor.hpp.
 */
inline constexpr uint16_t LEFT_POSITION_HIGH = 0x20A7;
inline constexpr uint16_t LEFT_POSITION_LOW = 0x20A8;

/**
 * Right encoder position.
 *
 * Address    : High word 0x20A9, low word 0x20AA
 * Function   : 0x03 Read Holding Registers
 * Type       : int32, signed (high word first, big-endian word order)
 * Units      : raw encoder counts (NOT radians)
 * Side       : RIGHT
 * Status     : PHYSICALLY_VERIFIED (register presence and word order)
 *              CALIBRATION_REQUIRED (counts-per-wheel-revolution scale)
 */
inline constexpr uint16_t RIGHT_POSITION_HIGH = 0x20A9;
inline constexpr uint16_t RIGHT_POSITION_LOW = 0x20AA;

/**
 * Left actual velocity.
 *
 * Address    : 0x20AB
 * Function   : 0x03 Read Holding Registers
 * Type       : int16, signed
 * Units      : RPM = raw_value / VELOCITY_SCALE_RPM
 * Side       : LEFT
 * Status     : PHYSICALLY_VERIFIED
 */
inline constexpr uint16_t LEFT_VELOCITY = 0x20AB;

/**
 * Right actual velocity.
 *
 * Address    : 0x20AC
 * Function   : 0x03 Read Holding Registers
 * Type       : int16, signed
 * Units      : RPM = raw_value / VELOCITY_SCALE_RPM
 * Side       : RIGHT
 * Status     : PHYSICALLY_VERIFIED
 */
inline constexpr uint16_t RIGHT_VELOCITY = 0x20AC;

inline constexpr double VELOCITY_SCALE_RPM = 10.0;

}  // namespace feedback


namespace status
{

/**
 * Driver status register.
 *
 * Address    : 0x20A2
 * Function   : 0x03 Read Holding Registers
 * Type       : uint16
 * Status     : PHYSICALLY_VERIFIED (register presence)
 *              UNVERIFIED (per-bit meaning)
 *
 * The manual documents 13 named fault conditions via front-panel LED
 * blink patterns (section 3.5) but does not publish a bit/value table
 * mapping this register's contents to those conditions. Only "raw value
 * is non-zero" is treated as meaningful (see driver_diagnostics.hpp).
 * Do not decode individual bits without a documented mapping.
 */
inline constexpr uint16_t DRIVER_STATUS = 0x20A2;

}  // namespace status


namespace fault
{

/**
 * Left motor fault register.
 *
 * Address    : 0x20A5
 * Function   : 0x03 Read Holding Registers
 * Type       : uint16
 * Side       : LEFT
 * Status     : PHYSICALLY_VERIFIED (register presence)
 *              UNVERIFIED (per-bit meaning - see status::DRIVER_STATUS)
 */
inline constexpr uint16_t LEFT_FAULT = 0x20A5;

/**
 * Right motor fault register.
 *
 * Address    : 0x20A6
 * Function   : 0x03 Read Holding Registers
 * Type       : uint16
 * Side       : RIGHT
 * Status     : PHYSICALLY_VERIFIED (register presence)
 *              UNVERIFIED (per-bit meaning - see status::DRIVER_STATUS)
 */
inline constexpr uint16_t RIGHT_FAULT = 0x20A6;

/**
 * No documented fault-clear / fault-reset register or command value
 * exists in the supplied manual or in bring-up notes. Do not invent one.
 * A fault condition must currently be cleared by a verified external
 * procedure (e.g. power cycle) - see ZlacDriveDriver and
 * zlac_drive/README.md.
 */

}  // namespace fault

}  // namespace zlac_drive::registers

#endif  // ZLAC_DRIVE__REGISTERS_HPP_