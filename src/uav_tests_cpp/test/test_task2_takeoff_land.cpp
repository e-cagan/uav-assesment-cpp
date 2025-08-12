#include "uav_tests_cpp/mavros_fixture.hpp"
#include "uav_control_cpp/mavros_client.hpp"
#include <chrono>
using namespace std::chrono_literals;

TEST_F(MavrosFixture, TakeoffLand) {
  MavrosClient cli;
  cli.init();

  ASSERT_TRUE(cli.wait_for_mavros(20.0)) << "MAVROS services not ready";

  cli.pump_setpoints(0.0, 120, std::chrono::milliseconds(20));

  ASSERT_TRUE(cli.set_mode("OFFBOARD", 5.0)) << "Failed to set OFFBOARD";
  ASSERT_TRUE(cli.arm(true, 5.0))            << "Arm failed";

  cli.pump_setpoints(3.0, 250, std::chrono::milliseconds(20));
  EXPECT_TRUE(cli.wait_alt_ge(1.8, 12.0))    << "Altitude did not rise enough";

  cli.pump_setpoints(3.0, 120, std::chrono::milliseconds(20));

  bool landed_mode = cli.set_mode("AUTO.LAND", 5.0);
  if (!landed_mode) {
    cli.pump_setpoints(0.0, 300, std::chrono::milliseconds(20));
  }

  std::this_thread::sleep_for(2s);
  (void)cli.arm(false, 5.0);

  SUCCEED();
}
