#ifndef ZLAC_DRIVE__DIAGNOSTICS__DRIVER_DIAGNOSTICS_HPP_
#define ZLAC_DRIVE__DIAGNOSTICS__DRIVER_DIAGNOSTICS_HPP_

#include <cstdint>

#include "zlac_drive/driver/driver_state.hpp"

namespace zlac_drive::diagnostics
{

/**
 * @brief Snapshot of ZLAC8015D driver and communication health.
 *
 * The ZLAC8015D user manual documents 13 named fault conditions via
 * front-panel LED blink codes (section 3.5) but does not publish a
 * register bit/value table mapping status/fault register contents to
 * those named conditions. This struct therefore exposes:
 *
 *   - the raw status/fault register values (for logging and any future
 *     mapping once verified), and
 *   - only the DERIVED boolean "some fault is active", which relies
 *     solely on "the fault register is non-zero when no fault exists"
 *     being false - a safe, conservative assumption for a fault
 *     register, but NOT a claim about which specific fault is active.
 *
 * Do not add per-bit fault decoding here without a documented (manual
 * or physically verified) bit map.
 */
struct DriverDiagnostics
{
  /**
   * Current driver state machine value (see driver/driver_state.hpp).
   */
  zlac_drive::driver::DriverState state{
    zlac_drive::driver::DriverState::DISCONNECTED};

  /**
   * True once at least one successful Modbus transaction has completed.
   */
  bool communication_ok{false};

  /**
   * Raw driver status register (registers::status::DRIVER_STATUS).
   * UNVERIFIED per-bit meaning - see registers.hpp.
   */
  uint16_t status_raw{0};

  /**
   * Raw left motor fault register (registers::fault::LEFT_FAULT).
   * UNVERIFIED per-bit meaning - see registers.hpp.
   */
  uint16_t left_fault_raw{0};

  /**
   * Raw right motor fault register (registers::fault::RIGHT_FAULT).
   * UNVERIFIED per-bit meaning - see registers.hpp.
   */
  uint16_t right_fault_raw{0};

  /**
   * @brief DERIVED: whether the left motor fault register is non-zero.
   */
  bool has_left_fault() const noexcept
  {
    return left_fault_raw != 0;
  }

  /**
   * @brief DERIVED: whether the right motor fault register is non-zero.
   */
  bool has_right_fault() const noexcept
  {
    return right_fault_raw != 0;
  }

  /**
   * @brief DERIVED: whether either verified motor fault register is set.
   *
   * The status register is informational; 0x4040 is the observed healthy
   * value on the verified ZLAC8015D controller and is not treated as a fault.
   */
  bool has_fault() const noexcept
  {
    return has_left_fault() || has_right_fault();
  }
};

}  // namespace zlac_drive::diagnostics

#endif  // ZLAC_DRIVE__DIAGNOSTICS__DRIVER_DIAGNOSTICS_HPP_
