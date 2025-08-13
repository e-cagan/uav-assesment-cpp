#include "uav_tests_cpp/mavros_fixture.hpp"
#include "uav_control_cpp/mavros_client.hpp"
#include <chrono>
#include <stdexcept>
using namespace std::chrono_literals;

TEST_F(MavrosFixture, ArmDisarm) {
  MavrosClient cli;

  // init
  try {
    cli.init();
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: init() - " << e.what();
  }

  // wait_for_mavros
  bool ready = false;
  try {
    ready = cli.wait_for_mavros(20.0);
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: wait_for_mavros() - " << e.what();
  }
  ASSERT_TRUE(ready) << "MAVROS services not ready in time";

  // arm(true)
  bool ok = false;
  try {
    ok = cli.arm(true, 5.0);
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: arm(true) - " << e.what();
  }
  EXPECT_TRUE(ok) << "Arm() failed";
  std::this_thread::sleep_for(1s);

  // arm(false)
  try {
    ok = cli.arm(false, 5.0);
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: arm(false) - " << e.what();
  }
  EXPECT_TRUE(ok) << "Disarm() failed";
}
