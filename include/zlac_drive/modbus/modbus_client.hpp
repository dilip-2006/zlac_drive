#ifndef ZLAC_DRIVE__MODBUS__MODBUS_CLIENT_HPP_
#define ZLAC_DRIVE__MODBUS__MODBUS_CLIENT_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "zlac_drive/modbus/modbus_types.hpp"

namespace zlac_drive::modbus
{

/**
 * @brief Modbus RTU client over a serial connection.
 *
 * This class implements generic Modbus RTU communication.
 * It intentionally contains no ZLAC8015D-specific register addresses
 * or command values.
 */
class ModbusClient
{
public:
  explicit ModbusClient(const SerialConfig & config);

  ~ModbusClient();

  ModbusClient(const ModbusClient &) = delete;
  ModbusClient & operator=(const ModbusClient &) = delete;

  ModbusClient(ModbusClient &&) = delete;
  ModbusClient & operator=(ModbusClient &&) = delete;

  /**
   * @brief Open and configure the serial connection.
   */
  void connect();

  /**
   * @brief Close the serial connection.
   */
  void disconnect();

  /**
   * @brief Check whether the serial connection is open.
   */
  bool is_connected() const noexcept;

  /**
   * @brief Read Modbus holding registers.
   *
   * @param slave_id Modbus slave address.
   * @param start_address First register address.
   * @param register_count Number of consecutive registers.
   *
   * @return Register values in response order.
   */
  std::vector<uint16_t> read_holding_registers(
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t register_count);

  /**
   * @brief Write a single Modbus holding register.
   */
  void write_single_register(
    uint8_t slave_id,
    uint16_t register_address,
    uint16_t value);

  /**
   * @brief Write multiple consecutive Modbus holding registers.
   */
  void write_multiple_registers(
    uint8_t slave_id,
    uint16_t start_address,
    const std::vector<uint16_t> & values);

  /**
   * @brief Calculate the Modbus RTU CRC-16.
   *
   * @param data Frame data excluding CRC bytes.
   *
   * @return Calculated CRC value.
   */
  static uint16_t calculate_crc(
    const std::vector<uint8_t> & data);

private:
  SerialConfig config_;

  int file_descriptor_{-1};

  /**
   * @brief Write a complete frame to the serial device.
   */
  void write_frame(
    const std::vector<uint8_t> & frame);

  /**
   * @brief Read exactly the requested number of bytes.
   */
  std::vector<uint8_t> read_exact(
    std::size_t byte_count);

  /**
   * @brief Send a Modbus request and receive the expected response.
   */
  std::vector<uint8_t> transact(
    const std::vector<uint8_t> & request,
    std::size_t normal_response_size);

  /**
   * @brief Append Modbus RTU CRC bytes to a frame.
   */
  static void append_crc(
    std::vector<uint8_t> & frame);

  /**
   * @brief Validate the CRC of a received frame.
   */
  static void validate_crc(
    const std::vector<uint8_t> & frame);

  /**
   * @brief Validate the Modbus response header.
   *
   * Checks slave ID, function code, and Modbus exception responses.
   */
  static void validate_response(
    const std::vector<uint8_t> & response,
    uint8_t expected_slave_id,
    FunctionCode expected_function);
};

}  // namespace zlac_drive::modbus

#endif  // ZLAC_DRIVE__MODBUS__MODBUS_CLIENT_HPP_