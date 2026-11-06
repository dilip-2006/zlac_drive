#ifndef ZLAC_DRIVE__MODBUS__MODBUS_EXCEPTION_HPP_
#define ZLAC_DRIVE__MODBUS__MODBUS_EXCEPTION_HPP_

#include <stdexcept>
#include <string>

namespace zlac_drive::modbus
{

/**
 * @brief Base exception for Modbus communication failures.
 */
class ModbusException : public std::runtime_error
{
public:
  explicit ModbusException(const std::string & message)
  : std::runtime_error(message)
  {
  }
};

/**
 * @brief Serial connection or transport failure.
 *
 * Examples:
 * - Serial device cannot be opened.
 * - Serial configuration fails.
 * - Read/write operation fails.
 * - Connection is unexpectedly lost.
 */
class ConnectionException : public ModbusException
{
public:
  explicit ConnectionException(const std::string & message)
  : ModbusException(message)
  {
  }
};

/**
 * @brief Modbus communication timed out.
 */
class TimeoutException : public ModbusException
{
public:
  explicit TimeoutException(const std::string & message)
  : ModbusException(message)
  {
  }
};

/**
 * @brief Modbus RTU CRC validation failed.
 */
class CrcException : public ModbusException
{
public:
  explicit CrcException(const std::string & message)
  : ModbusException(message)
  {
  }
};

/**
 * @brief Invalid or unexpected Modbus protocol response.
 *
 * Examples:
 * - Unexpected slave ID.
 * - Unexpected function code.
 * - Modbus exception response.
 * - Invalid response size.
 * - Invalid register byte count.
 */
class ProtocolException : public ModbusException
{
public:
  explicit ProtocolException(const std::string & message)
  : ModbusException(message)
  {
  }
};

}  // namespace zlac_drive::modbus

#endif  // ZLAC_DRIVE__MODBUS__MODBUS_EXCEPTION_HPP_