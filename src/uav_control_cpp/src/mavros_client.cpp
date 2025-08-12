#include "uav_control_cpp/mavros_client.hpp"
#include <thread>

using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("mavros_ctrl_cpp");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::msg::State &msg){ state_ = msg; });

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped &msg){ last_pose_ = msg; got_pose_ = true; });

  sp_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);

  arm_cli_  = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  mode_cli_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

bool MavrosClient::wait_for_mavros(double timeout_s) {
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (node_->now() < deadline) {
    if (arm_cli_->wait_for_service(500ms) && mode_cli_->wait_for_service(500ms))
      return true;
    exec_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool MavrosClient::arm(bool value, double timeout_s) {
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = value;
  auto fut = arm_cli_->async_send_request(req);
  auto rc = exec_->spin_until_future_complete(fut, std::chrono::duration<double>(timeout_s));
  return rc == rclcpp::FutureReturnCode::SUCCESS && fut.get()->success;
}

bool MavrosClient::set_mode(const std::string &mode, double timeout_s) {
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->base_mode = 0;
  req->custom_mode = mode;
  auto fut = mode_cli_->async_send_request(req);
  auto rc = exec_->spin_until_future_complete(fut, std::chrono::duration<double>(timeout_s));
  return rc == rclcpp::FutureReturnCode::SUCCESS && fut.get()->mode_sent;
}

void MavrosClient::pump_setpoints(double z, int count, std::chrono::milliseconds dt) {
  geometry_msgs::msg::PoseStamped sp;
  sp.header.frame_id = "map";
  sp.pose.position.x = 0;
  sp.pose.position.y = 0;
  sp.pose.position.z = z;
  for (int i = 0; i < count; ++i) {
    sp.header.stamp = node_->now();
    sp_pub_->publish(sp);
    exec_->spin_some();
    std::this_thread::sleep_for(dt);
  }
}

bool MavrosClient::wait_alt_ge(double alt, double timeout_s) {
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (node_->now() < deadline) {
    exec_->spin_some();
    if (got_pose_ && last_pose_.pose.position.z >= alt) return true;
    std::this_thread::sleep_for(50ms);
  }
  return false;
}
