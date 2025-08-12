cmake_minimum_required(VERSION 3.8)
project(uav_tests_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(uav_control_cpp REQUIRED)

ament_add_gtest(test_mavros_e2e tests/test_mavros_e2e.cpp)

# Link bağımlılıkları
ament_target_dependencies(test_mavros_e2e
  rclcpp geometry_msgs mavros_msgs uav_control_cpp
)

# Bu gtest'i otomatik koşturma, sadece binary üret
set_tests_properties(test_mavros_e2e PROPERTIES DISABLED TRUE)

# REPO_ROOT = repo kökü (paket kaynak dizininden iki yukarı: src/<pkg>/ -> repo root)
add_test(
  NAME e2e_with_stack
  COMMAND /usr/bin/bash -lc
  "REPO_ROOT=${CMAKE_SOURCE_DIR}/../.. ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_e2e.sh $<TARGET_FILE:test_mavros_e2e>"
)

install(PROGRAMS tests/run_e2e.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
