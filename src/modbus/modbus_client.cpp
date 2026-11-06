#include "zlac_drive/modbus/modbus_client.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <system_error>

#include "zlac_drive/modbus/modbus_exception.hpp"

namespace zlac_drive::modbus
{

namespace
{

speed_t baud_rate_to_termios(const uint32_t baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;

    case 19200:
      return B19200;

    case 38400:
      return B38400;

    case 57600:
      return B57600;

    case 115200:
      return B115200;

    default:
      throw ConnectionException(
              "Unsupported baud rate: " +
              std::to_string(baud_rate));
  }
}

std::string system_error_message(const std::string & operation)
{
  std::ostringstream stream;

  stream
    << operation
    << ": "
    << std::strerror(errno);

  return stream.str();
}

void append_uint16_be(
  std::vector<uint8_t> & frame,
  const uint16_t value)
{
  frame.push_back(
    static_cast<uint8_t>((value >> 8) & 0xFF));

  frame.push_back(
    static_cast<uint8_t>(value & 0xFF));
}

uint16_t read_uint16_be(
  const uint8_t high,
  const uint8_t low)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(high) << 8) |
    static_cast<uint16_t>(low));
}

}  // namespace


ModbusClient::ModbusClient(const SerialConfig & config)
: config_(config)
{
}

ModbusClient::~ModbusClient()
{
  disconnect();
}

void ModbusClient::connect()
{
  if (is_connected()) {
    return;
  }

  if (config_.device.empty()) {
    throw ConnectionException(
            "Serial device is empty");
  }

  file_descriptor_ = open(
    config_.device.c_str(),
    O_RDWR | O_NOCTTY | O_SYNC);

  if (file_descriptor_ < 0) {
    throw ConnectionException(
            system_error_message(
              "Failed to open serial device " +
              config_.device));
  }

  termios tty{};

  if (tcgetattr(file_descriptor_, &tty) != 0) {
    const auto message =
      system_error_message(
      "Failed to get serial configuration");

    disconnect();

    throw ConnectionException(message);
  }

  cfmakeraw(&tty);

  const speed_t baud_rate =
    baud_rate_to_termios(config_.baud_rate);

  if (
    cfsetispeed(&tty, baud_rate) != 0 ||
    cfsetospeed(&tty, baud_rate) != 0)
  {
    const auto message =
      system_error_message(
      "Failed to set serial baud rate");

    disconnect();

    throw ConnectionException(message);
  }

  tty.c_cflag |=
    CLOCAL |
    CREAD;

  tty.c_cflag &= ~CSIZE;

  switch (config_.data_bits) {
    case 7:
      tty.c_cflag |= CS7;
      break;

    case 8:
      tty.c_cflag |= CS8;
      break;

    default:
      disconnect();

      throw ConnectionException(
              "Unsupported data bits: " +
              std::to_string(config_.data_bits));
  }

  switch (config_.stop_bits) {
    case 1:
      tty.c_cflag &= ~CSTOPB;
      break;

    case 2:
      tty.c_cflag |= CSTOPB;
      break;

    default:
      disconnect();

      throw ConnectionException(
              "Unsupported stop bits: " +
              std::to_string(config_.stop_bits));
  }

  switch (config_.parity) {
    case Parity::NONE:
      tty.c_cflag &= ~PARENB;
      break;

    case Parity::EVEN:
      tty.c_cflag |= PARENB;
      tty.c_cflag &= ~PARODD;
      break;

    case Parity::ODD:
      tty.c_cflag |= PARENB;
      tty.c_cflag |= PARODD;
      break;
  }

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (
    tcsetattr(
      file_descriptor_,
      TCSANOW,
      &tty) != 0)
  {
    const auto message =
      system_error_message(
      "Failed to apply serial configuration");

    disconnect();

    throw ConnectionException(message);
  }

  tcflush(
    file_descriptor_,
    TCIOFLUSH);
}

void ModbusClient::disconnect()
{
  if (file_descriptor_ >= 0) {
    close(file_descriptor_);

    file_descriptor_ = -1;
  }
}

bool ModbusClient::is_connected() const noexcept
{
  return file_descriptor_ >= 0;
}

std::vector<uint16_t>
ModbusClient::read_holding_registers(
  const uint8_t slave_id,
  const uint16_t start_address,
  const uint16_t register_count)
{
  if (register_count == 0) {
    throw ProtocolException(
            "Register count must be greater than zero");
  }

  std::vector<uint8_t> request;

  request.reserve(8);

  request.push_back(slave_id);

  request.push_back(
    static_cast<uint8_t>(
      FunctionCode::READ_HOLDING_REGISTERS));

  append_uint16_be(
    request,
    start_address);

  append_uint16_be(
    request,
    register_count);

  append_crc(request);

  const std::size_t expected_response_size =
    5 +
    static_cast<std::size_t>(
    register_count) * 2;

  const auto response =
    transact(
    request,
    expected_response_size);

  if (response.size() != expected_response_size) {
    throw ProtocolException(
            "Unexpected response size");
  }

  if (response[0] != slave_id) {
    throw ProtocolException(
            "Unexpected slave ID in response");
  }

  const uint8_t expected_function =
    static_cast<uint8_t>(
    FunctionCode::READ_HOLDING_REGISTERS);

  if (response[1] != expected_function) {
    throw ProtocolException(
            "Unexpected function code in response");
  }

  const std::size_t byte_count =
    response[2];

  if (
    byte_count !=
    static_cast<std::size_t>(
      register_count) * 2)
  {
    throw ProtocolException(
            "Unexpected register byte count");
  }

  std::vector<uint16_t> values;

  values.reserve(register_count);

  for (uint16_t index = 0;
    index < register_count;
    ++index)
  {
    const std::size_t offset =
      3 +
      static_cast<std::size_t>(
      index) * 2;

    values.push_back(
      read_uint16_be(
        response[offset],
        response[offset + 1]));
  }

  return values;
}

void ModbusClient::write_single_register(
  const uint8_t slave_id,
  const uint16_t register_address,
  const uint16_t value)
{
  std::vector<uint8_t> request;

  request.reserve(8);

  request.push_back(slave_id);

  request.push_back(
    static_cast<uint8_t>(
      FunctionCode::WRITE_SINGLE_REGISTER));

  append_uint16_be(
    request,
    register_address);

  append_uint16_be(
    request,
    value);

  append_crc(request);

  const auto response =
    transact(
    request,
    request.size());

  if (response.size() != request.size()) {
    throw ProtocolException(
            "Unexpected write response size");
  }

  if (
    !std::equal(
      request.begin(),
      request.end() - 2,
      response.begin()))
  {
    throw ProtocolException(
            "Write single register response does not match request");
  }
}

void ModbusClient::write_multiple_registers(
  const uint8_t slave_id,
  const uint16_t start_address,
  const std::vector<uint16_t> & values)
{
  if (values.empty()) {
    throw ProtocolException(
            "Cannot write an empty register list");
  }

  if (values.size() > 123) {
    throw ProtocolException(
            "Too many registers in a single Modbus request");
  }

  std::vector<uint8_t> request;

  request.reserve(
    9 +
    values.size() * 2);

  request.push_back(slave_id);

  request.push_back(
    static_cast<uint8_t>(
      FunctionCode::WRITE_MULTIPLE_REGISTERS));

  append_uint16_be(
    request,
    start_address);

  append_uint16_be(
    request,
    static_cast<uint16_t>(
      values.size()));

  request.push_back(
    static_cast<uint8_t>(
      values.size() * 2));

  for (const uint16_t value : values) {
    append_uint16_be(
      request,
      value);
  }

  append_crc(request);

  constexpr std::size_t expected_response_size = 8;

  const auto response =
    transact(
    request,
    expected_response_size);

  if (response[0] != slave_id) {
    throw ProtocolException(
            "Unexpected slave ID in response");
  }

  const uint8_t expected_function =
    static_cast<uint8_t>(
    FunctionCode::WRITE_MULTIPLE_REGISTERS);

  if (response[1] != expected_function) {
    throw ProtocolException(
            "Unexpected function code in response");
  }

  const uint16_t response_address =
    read_uint16_be(
    response[2],
    response[3]);

  const uint16_t response_count =
    read_uint16_be(
    response[4],
    response[5]);

  if (response_address != start_address) {
    throw ProtocolException(
            "Unexpected starting address in response");
  }

  if (
    response_count !=
    static_cast<uint16_t>(
      values.size()))
  {
    throw ProtocolException(
            "Unexpected register count in response");
  }
}

uint16_t ModbusClient::calculate_crc(
  const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFF;

  for (const uint8_t byte : data) {
    crc ^= byte;

    for (uint8_t bit = 0;
      bit < 8;
      ++bit)
    {
      if (crc & 0x0001) {
        crc =
          static_cast<uint16_t>(
          (crc >> 1) ^
          0xA001);
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void ModbusClient::write_frame(
  const std::vector<uint8_t> & frame)
{
  if (!is_connected()) {
    throw ConnectionException(
            "Serial device is not connected");
  }

  std::size_t bytes_written = 0;

  while (bytes_written < frame.size()) {
    const ssize_t result =
      write(
      file_descriptor_,
      frame.data() + bytes_written,
      frame.size() - bytes_written);

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }

      throw ConnectionException(
              system_error_message(
                "Failed to write Modbus frame"));
    }

    if (result == 0) {
      throw ConnectionException(
              "Serial write returned zero bytes");
    }

    bytes_written +=
      static_cast<std::size_t>(
      result);
  }

  if (tcdrain(file_descriptor_) != 0) {
    throw ConnectionException(
            system_error_message(
              "Failed to drain serial output"));
  }
}

std::vector<uint8_t>
ModbusClient::read_exact(
  const std::size_t byte_count)
{
  if (!is_connected()) {
    throw ConnectionException(
            "Serial device is not connected");
  }

  std::vector<uint8_t> data(
    byte_count);

  std::size_t bytes_read = 0;

  const auto deadline =
    std::chrono::steady_clock::now() +
    config_.timeout;

  while (bytes_read < byte_count) {
    const auto now =
      std::chrono::steady_clock::now();

    if (now >= deadline) {
      throw TimeoutException(
              "Timed out waiting for Modbus response");
    }

    const auto remaining =
      std::chrono::duration_cast<
      std::chrono::milliseconds>(
      deadline - now);

    pollfd descriptor{};

    descriptor.fd =
      file_descriptor_;

    descriptor.events =
      POLLIN;

    const int poll_result =
      poll(
      &descriptor,
      1,
      static_cast<int>(
        remaining.count()));

    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }

      throw ConnectionException(
              system_error_message(
                "Serial poll failed"));
    }

    if (poll_result == 0) {
      throw TimeoutException(
              "Timed out waiting for Modbus response");
    }

    if (
      descriptor.revents &
      (POLLERR | POLLHUP | POLLNVAL))
    {
      throw ConnectionException(
              "Serial connection error");
    }

    const ssize_t result =
      read(
      file_descriptor_,
      data.data() + bytes_read,
      byte_count - bytes_read);

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }

      throw ConnectionException(
              system_error_message(
                "Failed to read Modbus response"));
    }

    if (result == 0) {
      continue;
    }

    bytes_read +=
      static_cast<std::size_t>(
      result);
  }

  return data;
}

std::vector<uint8_t>
ModbusClient::transact(
  const std::vector<uint8_t> & request,
  const std::size_t normal_response_size)
{
  if (!is_connected()) {
    throw ConnectionException(
      "Serial device is not connected");
  }

  if (request.size() < 2) {
    throw ProtocolException(
      "Invalid Modbus request");
  }

  if (normal_response_size < 5) {
    throw ProtocolException(
      "Invalid expected Modbus response size");
  }

  tcflush(
    file_descriptor_,
    TCIFLUSH);

  write_frame(request);

  // Read slave ID and function code first; this is enough to tell a
  // normal response (function code unchanged) from a Modbus exception
  // response (function code with the high bit set), which are shorter.
  auto response = read_exact(2);

  const uint8_t request_slave_id = request[0];

  const bool is_exception_response =
    response[0] == request_slave_id &&
    response[1] ==
    static_cast<uint8_t>(request[1] | 0x80);

  const std::size_t total_size =
    is_exception_response ?
    5 :
    normal_response_size;

  // We already read 2 bytes.
  const std::size_t remaining_size =
    total_size - response.size();

  const auto remaining =
    read_exact(remaining_size);

  response.insert(
    response.end(),
    remaining.begin(),
    remaining.end());

  validate_crc(response);

  const auto request_function =
    static_cast<FunctionCode>(request[1]);

  // Validates the header and throws ProtocolException for an exception
  // response (including reporting the Modbus exception code).
  validate_response(
    response,
    request_slave_id,
    request_function);

  return response;
}

void ModbusClient::validate_response(
  const std::vector<uint8_t> & response,
  const uint8_t expected_slave_id,
  const FunctionCode expected_function)
{
  if (response.size() < 4) {
    throw ProtocolException(
            "Modbus response is too short");
  }

  if (response[0] != expected_slave_id) {
    throw ProtocolException(
            "Unexpected slave ID in response");
  }

  const uint8_t expected_function_code =
    static_cast<uint8_t>(expected_function);

  if (
    response[1] ==
    static_cast<uint8_t>(
      expected_function_code | 0x80))
  {
    if (response.size() < 5) {
      throw ProtocolException(
              "Incomplete Modbus exception response");
    }

    throw ProtocolException(
            "Modbus exception code: " +
            std::to_string(response[2]));
  }

  if (response[1] != expected_function_code) {
    throw ProtocolException(
            "Unexpected function code in response");
  }
}

void ModbusClient::append_crc(
  std::vector<uint8_t> & frame)
{
  const uint16_t crc =
    calculate_crc(frame);

  frame.push_back(
    static_cast<uint8_t>(
      crc & 0xFF));

  frame.push_back(
    static_cast<uint8_t>(
      (crc >> 8) & 0xFF));
}

void ModbusClient::validate_crc(
  const std::vector<uint8_t> & frame)
{
  if (frame.size() < 3) {
    throw CrcException(
            "Frame too short for CRC validation");
  }

  const std::size_t data_size =
    frame.size() - 2;

  std::vector<uint8_t> data(
    frame.begin(),
    frame.begin() +
    static_cast<std::ptrdiff_t>(
      data_size));

  const uint16_t calculated_crc =
    calculate_crc(data);

  const uint16_t received_crc =
    static_cast<uint16_t>(
      frame[data_size]) |
    static_cast<uint16_t>(
      static_cast<uint16_t>(
        frame[data_size + 1]) << 8);

  if (calculated_crc != received_crc) {
    throw CrcException(
            "Modbus CRC validation failed");
  }
}

}  // namespace zlac_drive::modbus