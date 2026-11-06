cmake_minimum_required(VERSION 3.8)

project(zlac_drive)

# ============================================================
# C++ Standard
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Compiler Warnings
# ============================================================

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# ============================================================
# ZLAC Drive Library
# ============================================================

add_library(zlac_drive SHARED
  src/modbus/modbus_client.cpp
  src/driver/motor.cpp
  src/driver/zlac_drive_driver.cpp
  src/hardware/zlac_system_hardware.cpp
  src/diagnostics/driver_diagnostics.cpp
)

target_include_directories(zlac_drive
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(zlac_drive
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(zlac_drive
  hardware_interface
  pluginlib
  rclcpp
)

# ============================================================
# ros2_control Hardware Plugin
# ============================================================

pluginlib_export_plugin_description_file(
  hardware_interface
  zlac_drive_hardware_plugin.xml
)

# ============================================================
# Install Library
# ============================================================

install(
  TARGETS
    zlac_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================
# Install Headers
# ============================================================

install(
  DIRECTORY include/
  DESTINATION include
)

# ============================================================
# Install Plugin Description
# ============================================================

install(
  FILES
    zlac_drive_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)

  # ----------------------------------------------------------
  # Modbus tests
  # ----------------------------------------------------------

  ament_add_gtest(
    test_modbus_client
    test/test_modbus_client.cpp
  )

  if(TARGET test_modbus_client)

    target_link_libraries(
      test_modbus_client
      zlac_drive
    )

    target_include_directories(
      test_modbus_client
      PRIVATE
        include
    )

  endif()

  # ----------------------------------------------------------
  # Motor tests
  # ----------------------------------------------------------

  ament_add_gtest(
    test_motor
    test/test_motor.cpp
  )

  if(TARGET test_motor)

    target_link_libraries(
      test_motor
      zlac_drive
    )

    target_include_directories(
      test_motor
      PRIVATE
        include
    )

  endif()

  # ----------------------------------------------------------
  # Driver tests
  # ----------------------------------------------------------

  ament_add_gtest(
    test_driver
    test/test_driver.cpp
  )

  if(TARGET test_driver)

    target_link_libraries(
      test_driver
      zlac_drive
    )

    target_include_directories(
      test_driver
      PRIVATE
        include
    )

  endif()

  # ----------------------------------------------------------
  # Hardware Interface tests
  # ----------------------------------------------------------

  ament_add_gtest(
    test_hardware_interface
    test/test_hardware_interface.cpp
  )

  if(TARGET test_hardware_interface)

    target_link_libraries(
      test_hardware_interface
      zlac_drive
    )

    target_include_directories(
      test_hardware_interface
      PRIVATE
        include
    )

    ament_target_dependencies(
      test_hardware_interface
      hardware_interface
      rclcpp
    )

  endif()

endif()

# ============================================================
# Export
# ============================================================

ament_export_include_directories(
  include
)

ament_export_libraries(
  zlac_drive
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()