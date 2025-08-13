#include "uav_control_cpp/mavros_client.hpp"
#include <stdexcept>
#include <thread>
using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  // Bilerek boş: throw yok -> test devam etsin
}

bool MavrosClient::wait_for_mavros(double /*timeout_s*/) {
  // Ortam hazır varsay: TRUE döndür -> test devam etsin
  return true;
}

bool MavrosClient::arm(bool /*value*/, double /*timeout_s*/) {
  // Bilerek yanlış davranış: FALSE döndür -> Task1 FAIL olsun
  return false;
}

bool MavrosClient::set_mode(const std::string& /*mode*/, double /*timeout_s*/) {
  // Aday kodu eksik -> Task2'de ilk set_mode çağrısında SKIP'e düşsün
  throw std::logic_error("not implemented: set_mode");
}

void MavrosClient::pump_setpoints(double /*z*/, int /*count*/, std::chrono::milliseconds /*dt*/) {
  // Task2'nin OFFBOARD ön-ısınma adımında SKIP'e düşürmek için
  throw std::logic_error("not implemented: pump_setpoints");
}

bool MavrosClient::wait_alt_ge(double /*alt*/, double /*timeout_s*/) {
  // Task2 bu aşamaya gelirse yine SKIP'e düşsün
  throw std::logic_error("not implemented: wait_alt_ge");
}
