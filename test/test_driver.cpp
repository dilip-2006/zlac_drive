#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

#include "zlac_drive/driver/driver_state.hpp"
#include "zlac_drive/driver/motor.hpp"
#include "zlac_drive/driver/zlac_drive_driver.hpp"
#include "zlac_drive/modbus/modbus_types.hpp"

namespace zlac_drive::driver
{

TEST(ZlacDriveConfigTest, DefaultsRequireExplicitSlaveVerification)
{
  const ZlacDriveConfig config;
  EXPECT_EQ(config.slave_id, 1U);
  EXPECT_FALSE(config.slave_id_verified);
}

TEST(ZlacDriveConfigTest, DefaultsToPeakRpmAndNoInversion)
{
  const ZlacDriveConfig config;
  EXPECT_DOUBLE_EQ(config.max_velocity_rpm, MotorLimits::kPeakRpm);
  EXPECT_FALSE(config.left_motor_inverted);
  EXPECT_FALSE(config.right_motor_inverted);
}

TEST(ZlacDriveDriverTest, IsNonCopyable)
{
  EXPECT_FALSE(std::is_copy_constructible_v<ZlacDriveDriver>);
  EXPECT_FALSE(std::is_copy_assignable_v<ZlacDriveDriver>);
}

TEST(ZlacDriveDriverTest, RejectsMaxVelocityAboveMotorPeak)
{
  ZlacDriveConfig config;
  config.max_velocity_rpm = MotorLimits::kPeakRpm + 1.0;

  const modbus::SerialConfig serial_config;

  EXPECT_THROW(
    ZlacDriveDriver(serial_config, config),
    std::invalid_argument);
}

TEST(ZlacDriveDriverTest, RejectsNonPositiveMaxVelocity)
{
  ZlacDriveConfig config;
  config.max_velocity_rpm = 0.0;

  const modbus::SerialConfig serial_config;

  EXPECT_THROW(
    ZlacDriveDriver(serial_config, config),
    std::invalid_argument);
}

TEST(ZlacDriveDriverTest, StartsDisconnected)
{
  const modbus::SerialConfig serial_config;
  const ZlacDriveConfig driver_config;

  ZlacDriveDriver driver(serial_config, driver_config);

  EXPECT_FALSE(driver.is_connected());
  EXPECT_EQ(driver.state(), DriverState::DISCONNECTED);
}

// ==============================================================
// Safety-limit pipeline (pure functions, no hardware required).
// ==============================================================

TEST(ClampVelocityRpmTest, PassesValuesWithinRange)
{
  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(100.0, 350.0),
    100.0);

  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(-100.0, 350.0),
    -100.0);

  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(0.0, 350.0),
    0.0);
}

TEST(ClampVelocityRpmTest, ClampsPositiveOverLimit)
{
  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(1000.0, 350.0),
    350.0);
}

TEST(ClampVelocityRpmTest, ClampsNegativeOverLimit)
{
  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(-1000.0, 350.0),
    -350.0);
}

TEST(ClampVelocityRpmTest, ClampsAtExactLimit)
{
  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(350.0, 350.0),
    350.0);

  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(-350.0, 350.0),
    -350.0);
}

TEST(ClampVelocityRpmTest, TreatsNonFiniteInputAsZero)
{
  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(
      std::numeric_limits<double>::quiet_NaN(), 350.0),
    0.0);

  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(
      std::numeric_limits<double>::infinity(), 350.0),
    0.0);

  EXPECT_DOUBLE_EQ(
    ZlacDriveDriver::clamp_velocity_rpm(
      -std::numeric_limits<double>::infinity(), 350.0),
    0.0);
}

TEST(ToVelocityRegisterValueTest, RoundsToNearestInteger)
{
  EXPECT_EQ(ZlacDriveDriver::to_velocity_register_value(100.4), 100);
  EXPECT_EQ(ZlacDriveDriver::to_velocity_register_value(100.6), 101);
  EXPECT_EQ(ZlacDriveDriver::to_velocity_register_value(-100.6), -101);
}

TEST(ToVelocityRegisterValueTest, HandlesMaxMotorPeakRpm)
{
  EXPECT_EQ(
    ZlacDriveDriver::to_velocity_register_value(MotorLimits::kPeakRpm),
    350);

  EXPECT_EQ(
    ZlacDriveDriver::to_velocity_register_value(-MotorLimits::kPeakRpm),
    -350);
}

TEST(ToVelocityRegisterValueTest, RejectsValueOutsideInt16Range)
{
  EXPECT_THROW(
    ZlacDriveDriver::to_velocity_register_value(40000.0),
    std::out_of_range);

  EXPECT_THROW(
    ZlacDriveDriver::to_velocity_register_value(-40000.0),
    std::out_of_range);
}

}  // namespace zlac_drive::driver
