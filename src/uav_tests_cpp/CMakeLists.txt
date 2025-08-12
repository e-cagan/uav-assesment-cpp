cmake_minimum_required(VERSION 3.8)
project(uav_tests_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_control_cpp REQUIRED)  # ADAY KODU

include_directories(include)

# ---- TEST BINARIES ----
ament_add_gtest(test_task1_arm_disarm
  test/test_task1_arm_disarm.cpp
)
ament_target_dependencies(test_task1_arm_disarm
  rclcpp mavros_msgs geometry_msgs uav_control_cpp
)

ament_add_gtest(test_task2_takeoff_land
  test/test_task2_takeoff_land.cpp
)

set_tests_properties(test_task1_arm_disarm PROPERTIES DISABLED TRUE)
set_tests_properties(test_task2_takeoff_land PROPERTIES DISABLED TRUE)

ament_target_dependencies(test_task2_takeoff_land
  rclcpp mavros_msgs geometry_msgs uav_control_cpp
)

# ---- E2E KOŞU (SITL + MAVROS + GTEST) ----
# repo kökü = paket kaynak dizininden iki yukarı
set(REPO_ROOT_PATH "${CMAKE_SOURCE_DIR}/../..")

add_test(
  NAME task1_arm_disarm
  COMMAND /usr/bin/bash -lc
  "REPO_ROOT=${REPO_ROOT_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_e2e.sh $<TARGET_FILE:test_task1_arm_disarm>"
)

add_test(
  NAME task2_takeoff_land
  COMMAND /usr/bin/bash -lc
  "REPO_ROOT=${REPO_ROOT_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_e2e.sh $<TARGET_FILE:test_task2_takeoff_land>"
)

install(PROGRAMS tests/run_e2e.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
