#include "uav_tests_cpp/mavros_fixture.hpp"
#include "uav_control_cpp/mavros_client.hpp"
#include <chrono>
#include <stdexcept>
using namespace std::chrono_literals;

TEST_F(MavrosFixture, TakeoffLand) {
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

  if (!ready) {
    GTEST_SKIP() << "MAVROS services not ready";
  }

  // OFFBOARD ön ısınma setpoint'leri
  try {
    cli.pump_setpoints(0.0, 120, std::chrono::milliseconds(20));
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: pump_setpoints() - " << e.what();
  }

  // mode -> OFFBOARD
  bool ok = false;
  try {
    ok = cli.set_mode("OFFBOARD", 5.0);
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: set_mode(OFFBOARD) - " << e.what();
  }
  ASSERT_TRUE(ok) << "Failed to set OFFBOARD";

  // arm
  try {
    ok = cli.arm(true, 5.0);
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: arm(true) - " << e.what();
  }
  ASSERT_TRUE(ok) << "Arm failed";

  // takeoff (hedef ~3m)
  try {
    cli.pump_setpoints(3.0, 250, std::chrono::milliseconds(20));
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: pump_setpoints() - " << e.what();
  }

  bool alt_ok = false;
  try {
    alt_ok = cli.wait_alt_ge(1.8, 12.0);
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: wait_alt_ge() - " << e.what();
  }
  EXPECT_TRUE(alt_ok) << "Altitude did not rise enough";

  // biraz hover
  try {
    cli.pump_setpoints(3.0, 120, std::chrono::milliseconds(20));
  } catch (const std::logic_error& e) {
    GTEST_SKIP() << "Candidate not implemented: pump_setpoints() - " << e.what();
  }

  // land: PX4'te AUTO.LAND; olmazsa setpoint'le alçalt
  bool landed_mode = false;
  try {
    landed_mode = cli.set_mode("AUTO.LAND", 5.0);
  } catch (const std::logic_error& e) {
    // AUTO.LAND yoksa setpoint ile iniş dene
    try { cli.pump_setpoints(0.0, 300, std::chrono::milliseconds(20)); }
    catch (const std::logic_error&) {
      GTEST_SKIP() << "Candidate not implemented: set_mode()/pump_setpoints() - " << e.what();
    }
  }
  if (!landed_mode) {
    try { cli.pump_setpoints(0.0, 300, std::chrono::milliseconds(20)); }
    catch (const std::logic_error& e) {
      GTEST_SKIP() << "Candidate not implemented: pump_setpoints() - " << e.what();
    }
  }

  std::this_thread::sleep_for(2s);
  try { (void)cli.arm(false, 5.0); } catch (...) {}

  SUCCEED();
}
