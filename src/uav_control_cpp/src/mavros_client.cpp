#include "uav_control_cpp/mavros_client.hpp"
#include <thread>
#include <stdexcept>

using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  // Sadece ROS node/executor oluştur – gerçek MAVROS bağları yok.
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
  node_ = rclcpp::Node::make_shared("dummy_mavros_client");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
}

bool MavrosClient::wait_for_mavros(double /*timeout_s*/) {
  // “Hazır” gibi davran
  return true;
}

bool MavrosClient::arm(bool /*value*/, double /*timeout_s*/) {
  // BİLEREK BAŞARISIZ: Task1 FAIL etsin
  return false;
}

bool MavrosClient::set_mode(const std::string& /*mode*/, double /*timeout_s*/) {
  // Task2’yi SKIP’e düşürmek için not-implemented
  throw std::logic_error("TODO: implement set_mode()");
}

void MavrosClient::pump_setpoints(double /*z*/, int /*count*/, std::chrono::milliseconds /*dt*/) {
  throw std::logic_error("TODO: implement pump_setpoints()");
}

bool MavrosClient::wait_alt_ge(double /*alt*/, double /*timeout_s*/) {
  throw std::logic_error("TODO: implement wait_alt_ge()");
}
