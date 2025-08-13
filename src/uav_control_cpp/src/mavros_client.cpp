#include "uav_control_cpp/mavros_client.hpp"
#include <stdexcept>
#include <thread>
using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  // THROW YOK -> test devam etsin
}

bool MavrosClient::wait_for_mavros(double /*timeout_s*/) {
  // TRUE -> ortam hazır varsay, SKIP'e düşmesin
  return true;
}

bool MavrosClient::arm(bool /*value*/, double /*timeout_s*/) {
  // BİLEREK HATALI: FALSE -> Task-1 FAIL
  return false;
}

bool MavrosClient::set_mode(const std::string& /*mode*/, double /*timeout_s*/) {
  // Implement edilmedi -> Task-2'de ilk set_mode çağrısında SKIP
  throw std::logic_error("not implemented: set_mode");
}

void MavrosClient::pump_setpoints(double /*z*/, int /*count*/, std::chrono::milliseconds /*dt*/) {
  // Task-2'de OFFBOARD ön-ısınmada SKIP'e düşürmek için
  throw std::logic_error("not implemented: pump_setpoints");
}

bool MavrosClient::wait_alt_ge(double /*alt*/, double /*timeout_s*/) {
  // Task-2 bu aşamaya gelirse de SKIP sebebi olsun
  throw std::logic_error("not implemented: wait_alt_ge");
}
