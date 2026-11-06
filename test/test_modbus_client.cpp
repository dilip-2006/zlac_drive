#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "zlac_drive/modbus/modbus_client.hpp"
#include "zlac_drive/modbus/modbus_exception.hpp"
#include "zlac_drive/modbus/modbus_types.hpp"

namespace zlac_drive::modbus
{

TEST(ModbusClientTest, CalculatesKnownModbusCrc)
{
  const std::vector<uint8_t> frame{
    0x01,
    0x03,
    0x00,
    0x00,
    0x00,
    0x01
  };

  const uint16_t crc =
    ModbusClient::calculate_crc(frame);

  EXPECT_EQ(crc, 0x0A84);
}

TEST(ModbusTypesTest, DefaultSerialConfig)
{
  const SerialConfig config;

  EXPECT_TRUE(config.device.empty());

  EXPECT_EQ(
    config.baud_rate,
    115200U);

  EXPECT_EQ(
    config.data_bits,
    8U);

  EXPECT_EQ(
    config.stop_bits,
    1U);

  EXPECT_EQ(
    config.parity,
    Parity::NONE);

  EXPECT_EQ(
    config.timeout,
    std::chrono::milliseconds(100));
}

TEST(ModbusClientTest, StartsDisconnected)
{
  SerialConfig config;

  config.device = "/dev/does_not_exist";

  ModbusClient client(config);

  EXPECT_FALSE(
    client.is_connected());
}

TEST(ModbusClientTest, RejectsEmptySerialDevice)
{
  SerialConfig config;

  config.device = "";

  ModbusClient client(config);

  EXPECT_THROW(
    client.connect(),
    ConnectionException);

  EXPECT_FALSE(
    client.is_connected());
}

TEST(ModbusClientTest, RejectsZeroRegisterRead)
{
  SerialConfig config;

  ModbusClient client(config);

  EXPECT_THROW(
    client.read_holding_registers(
      1,
      0,
      0),
    ProtocolException);
}

TEST(ModbusClientTest, RejectsEmptyMultipleRegisterWrite)
{
  SerialConfig config;

  ModbusClient client(config);

  const std::vector<uint16_t> values;

  EXPECT_THROW(
    client.write_multiple_registers(
      1,
      0,
      values),
    ProtocolException);
}

TEST(ModbusClientTest, RejectsTooManyRegisters)
{
  SerialConfig config;

  ModbusClient client(config);

  std::vector<uint16_t> values(
    124,
    0);

  EXPECT_THROW(
    client.write_multiple_registers(
      1,
      0,
      values),
    ProtocolException);
}

}  // namespace zlac_drive::modbus