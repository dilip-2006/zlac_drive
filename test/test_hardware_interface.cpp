#include <gtest/gtest.h>
#include <type_traits>

#include "hardware_interface/hardware_info.hpp"
#include "zlac_drive/hardware/zlac_system_hardware.hpp"

namespace zlac_drive::hardware
{

TEST(ZlacSystemHardwareTest, IsDefaultConstructible)
{
  EXPECT_TRUE(std::is_default_constructible_v<ZlacSystemHardware>);
}

TEST(ZlacSystemHardwareTest, RejectsHardwareInfoWithoutTwoJoints)
{
  ZlacSystemHardware hardware;

  // A default-constructed HardwareInfo has zero joints, which does not
  // satisfy the "exactly 2 wheel joints" requirement enforced in
  // on_init(). This exercises the joint-count validation path without
  // needing a real serial device.
  const hardware_interface::HardwareInfo info{};

  EXPECT_EQ(
    hardware.on_init(info),
    hardware_interface::CallbackReturn::ERROR);
}

}  // namespace zlac_drive::hardware
