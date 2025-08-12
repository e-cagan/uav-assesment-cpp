#include <gtest/gtest.h>
#include "uav_control_cpp/mavros_client.hpp"

using namespace std::chrono_literals;

class MavrosFixture : public ::testing::Test {
protected:
  void SetUp() override {
    client_.init();
  }
  MavrosClient client_;
};

TEST_F(MavrosFixture, ArmDisarm) {
  ASSERT_TRUE(client_.wait_for_mavros()) << "MAVROS services not ready";
  EXPECT_TRUE(client_.arm(true)) << "Arm failed";
  std::this_thread::sleep_for(2s);
  EXPECT_TRUE(client_.arm(false)) << "Disarm failed";
}

TEST_F(MavrosFixture, TakeoffLand) {
  ASSERT_TRUE(client_.wait_for_mavros()) << "MAVROS services not ready";

  // OFFBOARD öncesi setpoint bas
  client_.pump_setpoints(0.0, 80);
  ASSERT_TRUE(client_.set_mode("OFFBOARD")) << "Failed to switch to OFFBOARD";
  ASSERT_TRUE(client_.arm(true)) << "Arm failed";

  // 3 m'e kalk
  client_.pump_setpoints(3.0, 250);
  EXPECT_TRUE(client_.wait_alt_ge(1.8, 8.0)) << "Altitude didn't rise as expected";

  // Hover
  client_.pump_setpoints(3.0, 150);

  // Land
  bool landed_via_mode = client_.set_mode("AUTO.LAND");
  if (!landed_via_mode) {
    client_.pump_setpoints(0.0, 300);
  }
  std::this_thread::sleep_for(2s);
  (void)client_.arm(false);
  SUCCEED();
}
