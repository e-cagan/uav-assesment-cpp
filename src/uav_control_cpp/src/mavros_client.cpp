#include "uav_control_cpp/mavros_client.hpp"
#include <thread>
#include <stdexcept>

using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  throw std::logic_error("TODO: implement init()");
}

bool MavrosClient::wait_for_mavros(double) {
  throw std::logic_error("TODO: implement wait_for_mavros()");
}

bool MavrosClient::arm(bool, double) {
  throw std::logic_error("TODO: implement arm()");
}

bool MavrosClient::set_mode(const std::string &, double) {
  throw std::logic_error("TODO: implement set_mode()");
}

void MavrosClient::pump_setpoints(double, int, std::chrono::milliseconds) {
  throw std::logic_error("TODO: implement pump_setpoints()");
}

bool MavrosClient::wait_alt_ge(double, double) {
  throw std::logic_error("TODO: implement wait_alt_ge()");
}
