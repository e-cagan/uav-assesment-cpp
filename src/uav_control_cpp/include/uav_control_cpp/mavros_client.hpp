#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

class MavrosClient {
public:
  MavrosClient();
  void init();
  bool wait_for_mavros(double timeout_s = 20.0);
  bool arm(bool value, double timeout_s = 5.0);
  bool set_mode(const std::string &mode, double timeout_s = 5.0);

  // Aynı imzayı koruyoruz ama akışı kalıcı bir timer ile sürdürüyoruz
  void pump_setpoints(double z, int count = 100,
                      std::chrono::milliseconds dt = std::chrono::milliseconds(50));

  bool wait_alt_ge(double alt, double timeout_s = 10.0);

  rclcpp::Node::SharedPtr node() const { return node_; }
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec() const { return exec_; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sp_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_cli_;

  // Kalıcı setpoint akışı
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds period_{50};
  double target_z_{0.0};

  // Durum
  std::atomic<bool> got_state_{false};
  std::atomic<bool> got_pose_{false};
  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped last_pose_;

  void ensure_stream_(std::chrono::milliseconds dt);
};
