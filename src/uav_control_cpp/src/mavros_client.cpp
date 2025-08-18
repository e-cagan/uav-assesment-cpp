#include "uav_control_cpp/mavros_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  node_ = std::make_shared<rclcpp::Node>("uav_control_client");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::msg::State &msg){ state_ = msg; });

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped &msg){
        last_pose_ = msg; got_pose_.store(true);
      });

  sp_pub_   = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);

  arm_cli_  = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  mode_cli_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

bool MavrosClient::wait_for_mavros(double timeout_s) {
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (rclcpp::ok() && node_->now() < deadline) {
    exec_->spin_some();
    // servislerin gelmesini bekle
    if (arm_cli_->wait_for_service(0ms) && mode_cli_->wait_for_service(0ms)) {
      // topic’ler akıyor mu? en azından state aldıysak kabul
      return true;
    }
    rclcpp::sleep_for(100ms);
  }
  return false;
}

bool MavrosClient::arm(bool value, double timeout_s) {
  if (!arm_cli_->wait_for_service(1s)) return false;
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = value;
  auto fut = arm_cli_->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready) return false;
  return fut.get()->success;
}

bool MavrosClient::set_mode(const std::string& mode, double timeout_s) {
  if (!mode_cli_->wait_for_service(1s)) return false;
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = mode;
  auto fut = mode_cli_->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready) return false;
  return fut.get()->mode_sent;
}

void MavrosClient::pump_setpoints(double z, int count, std::chrono::milliseconds dt) {
  // son poz varsa onu referans al; yoksa 0,0’dan gönder
  geometry_msgs::msg::PoseStamped sp;
  sp.header.frame_id = "map";
  sp.pose.position.x = got_pose_ ? last_pose_.pose.position.x : 0.0;
  sp.pose.position.y = got_pose_ ? last_pose_.pose.position.y : 0.0;
  sp.pose.position.z = z;
  sp.pose.orientation.w = 1.0;

  for (int i=0; i<count && rclcpp::ok(); ++i) {
    sp.header.stamp = node_->now();
    sp_pub_->publish(sp);
    exec_->spin_some();
    rclcpp::sleep_for(dt);
  }
}

bool MavrosClient::wait_alt_ge(double alt, double timeout_s) {
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (rclcpp::ok() && node_->now() < deadline) {
    exec_->spin_some();
    if (got_pose_.load() && last_pose_.pose.position.z >= alt) return true;
    rclcpp::sleep_for(50ms);
  }
  return false;
}