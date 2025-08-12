#include "uav_tests_cpp/mavros_fixture.hpp"
#include "uav_control_cpp/mavros_client.hpp"
#include <chrono>
using namespace std::chrono_literals;

TEST_F(MavrosFixture, ArmDisarm) {
  MavrosClient cli;
  cli.init();

  ASSERT_TRUE(cli.wait_for_mavros(20.0)) << "MAVROS services not ready in time";

  // Arm
  EXPECT_TRUE(cli.arm(true, 5.0)) << "Arm() failed";

  std::this_thread::sleep_for(1s);

  // Disarm
  EXPECT_TRUE(cli.arm(false, 5.0)) << "Disarm() failed";
}
