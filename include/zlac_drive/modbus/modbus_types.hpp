#ifndef ZLAC_DRIVE__MODBUS__MODBUS_TYPES_HPP_
#define ZLAC_DRIVE__MODBUS__MODBUS_TYPES_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace zlac_drive::modbus
{

/**
 * @brief Serial parity configuration.
 */
enum class Parity
{
  NONE,
  EVEN,
  ODD
};

/**
 * @brief RS-485 / serial connection configuration.
 *
 * This structure contains transport-layer configuration only.
 * It intentionally contains no ZLAC8015D-specific protocol settings.
 */
struct SerialConfig
{
  std::string device;

  uint32_t baud_rate{115200};

  uint8_t data_bits{8};

  uint8_t stop_bits{1};

  Parity parity{Parity::NONE};

  std::chrono::milliseconds timeout{100};
};

/**
 * @brief Supported Modbus RTU function codes.
 */
enum class FunctionCode : uint8_t
{
  READ_HOLDING_REGISTERS = 0x03,

  WRITE_SINGLE_REGISTER = 0x06,

  WRITE_MULTIPLE_REGISTERS = 0x10
};

}  // namespace zlac_drive::modbus

#endif  // ZLAC_DRIVE__MODBUS__MODBUS_TYPES_HPP_