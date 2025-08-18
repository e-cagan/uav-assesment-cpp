// File: src/mavros_client.cpp
#include "uav_control_cpp/mavros_client.hpp"
#include <thread>

using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);

  node_ = std::make_shared<rclcpp::Node>("uav_control_client");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  // QoS: pose -> SensorDataQoS (BestEffort); state -> BestEffort
  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).best_effort(),
      [this](mavros_msgs::msg::State::SharedPtr msg) {
        state_ = *msg;
        got_state_.store(true);
      });

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_pose_ = *msg;
        got_pose_.store(true);
      });

  sp_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());

  arm_cli_  = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  mode_cli_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

bool MavrosClient::wait_for_mavros(double timeout_s) {
  const auto deadline_ros  = node_->get_clock()->now() + rclcpp::Duration::from_seconds(timeout_s);
  const auto deadline_wall = std::chrono::steady_clock::now() + std::chrono::milliseconds((int)(timeout_s * 1000));

  while (rclcpp::ok()
         && node_->get_clock()->now() < deadline_ros
         && std::chrono::steady_clock::now() < deadline_wall) {

    // discovery’yi tetikle
    (void)arm_cli_->wait_for_service(200ms);
    (void)mode_cli_->wait_for_service(200ms);

    exec_->spin_some();

    const bool services_ready = arm_cli_->service_is_ready() && mode_cli_->service_is_ready();
    const bool state_ok       = got_state_.load();

    if (services_ready && state_ok) return true;

    rclcpp::sleep_for(100ms);
  }
  return false;
}

bool MavrosClient::arm(bool value, double timeout_s) {
  if (!arm_cli_->wait_for_service(1s)) return false;
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = value;
  auto fut = arm_cli_->async_send_request(req);

  auto rc = rclcpp::spin_until_future_complete(node_, fut,
            std::chrono::duration<double>(timeout_s));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) return false;
  return fut.get()->success;
}

bool MavrosClient::set_mode(const std::string &mode, double timeout_s) {
  if (!mode_cli_->wait_for_service(1s)) return false;
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->base_mode   = 0;
  req->custom_mode = mode;
  auto fut = mode_cli_->async_send_request(req);

  auto rc = rclcpp::spin_until_future_complete(node_, fut,
            std::chrono::duration<double>(timeout_s));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) return false;
  return fut.get()->mode_sent;
}

void MavrosClient::pump_setpoints(double z, int warmup_count, std::chrono::milliseconds dt) {
  sp_target_z_ = z;

  // timer yoksa oluştur; varsa period değiştiyse yeniden kur
  if (!sp_timer_ || sp_period_ != dt || sp_timer_->is_canceled()) {
    sp_period_ = dt;
    sp_timer_.reset();
    sp_timer_ = node_->create_wall_timer(sp_period_, [this]() {
      geometry_msgs::msg::PoseStamped sp;
      sp.header.stamp = node_->get_clock()->now();
      sp.header.frame_id = "map";
      sp.pose.orientation.w = 1.0;
      sp.pose.position.x = got_pose_ ? last_pose_.pose.position.x : 0.0;
      sp.pose.position.y = got_pose_ ? last_pose_.pose.position.y : 0.0;
      sp.pose.position.z = sp_target_z_;
      sp_pub_->publish(sp);
    });
  }

  // OFFBOARD öncesi ısınma
  for (int i = 0; i < warmup_count && rclcpp::ok(); ++i) {
    exec_->spin_some();
    rclcpp::sleep_for(dt);
  }
}

bool MavrosClient::wait_alt_ge(double alt, double timeout_s) {
  const auto deadline = node_->get_clock()->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (rclcpp::ok() && node_->get_clock()->now() < deadline) {
    exec_->spin_some();
    if (got_pose_.load() && last_pose_.pose.position.z >= alt) return true;
    rclcpp::sleep_for(50ms);
  }
  return false;
}
