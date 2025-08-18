#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

class MavrosClient {
public:
  MavrosClient();

  // Node & comms kurulum
  void init();

  // MAVROS servisleri + /mavros/state akışını bekler
  bool wait_for_mavros(double timeout_s = 20.0);

  // Arm / Disarm
  bool arm(bool value, double timeout_s = 5.0);

  // Mod değiştir ("OFFBOARD", "AUTO.LAND" ...)
  bool set_mode(const std::string &mode, double timeout_s = 5.0);

  // OFFBOARD için setpoint akışı: timer ile sürekli yayına geçer.
  // warmup_count: OFFBOARD öncesi kaç örnek bekleyelim (akışın başladığından emin olmak için)
  void pump_setpoints(double z,
                      int warmup_count = 100,
                      std::chrono::milliseconds dt = std::chrono::milliseconds(50));

  // Yerel ENU z >= alt olana kadar bekle
  bool wait_alt_ge(double alt, double timeout_s = 10.0);

  // Erişimciler
  rclcpp::Node::SharedPtr node() const { return node_; }
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec() const { return exec_; }

private:
  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;

  // IO
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sp_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_cli_;

  // Sürekli setpoint akışı için timer
  rclcpp::TimerBase::SharedPtr sp_timer_;
  std::chrono::milliseconds sp_period_{50};
  double sp_target_z_{0.0};

  // Durum
  std::atomic<bool> got_state_{false};
  std::atomic<bool> got_pose_{false};
  mavros_msgs::msg::State state_{};
  geometry_msgs::msg::PoseStamped last_pose_{};
};