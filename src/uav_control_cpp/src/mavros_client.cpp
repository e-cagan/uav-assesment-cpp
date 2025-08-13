#include "uav_control_cpp/mavros_client.hpp"
#include "uav_control_cpp/not_implemented.hpp"   // <-- EKLENDİ
#include <thread>
#include <stdexcept>

using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  throw uav::NotImplemented("init()");
}
bool MavrosClient::wait_for_mavros(double) {
  throw uav::NotImplemented("wait_for_mavros()");
}
bool MavrosClient::arm(bool, double) {
  throw uav::NotImplemented("arm()");
}
bool MavrosClient::set_mode(const std::string &, double) {
  throw uav::NotImplemented("set_mode()");
}
void MavrosClient::pump_setpoints(double, int, std::chrono::milliseconds) {
  throw uav::NotImplemented("pump_setpoints()");
}
bool MavrosClient::wait_alt_ge(double, double) {
  throw uav::NotImplemented("wait_alt_ge()");
}
