#include "uav_control_cpp/mavros_client.hpp"
using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("uav_control_client");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  // --- QoS: PUBLISHER = reliable; POSE SUBSCRIBER = SensorDataQoS (best effort) ---
  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", rclcpp::QoS(10).best_effort(),
    [this](const mavros_msgs::msg::State &msg){ state_ = msg; });

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped &msg){
        last_pose_ = msg; got_pose_.store(true);
      });

  sp_pub_   = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());

  arm_cli_  = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  mode_cli_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

bool MavrosClient::wait_for_mavros(double timeout_s) {
  const auto deadline = node_->get_clock()->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (rclcpp::ok() && node_->get_clock()->now() < deadline) {
    exec_->spin_some();
    const bool services_ready =
        arm_cli_->service_is_ready() && mode_cli_->service_is_ready();
    if (services_ready && got_state_.load()) return true;
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
            rclcpp::Duration::from_seconds(timeout_s));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) return false;
  return fut.get()->success;
}

bool MavrosClient::set_mode(const std::string& mode, double timeout_s) {
  if (!mode_cli_->wait_for_service(1s)) return false;
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->base_mode = 0;
  req->custom_mode = mode;
  auto fut = mode_cli_->async_send_request(req);
  auto rc = rclcpp::spin_until_future_complete(node_, fut,
            rclcpp::Duration::from_seconds(timeout_s));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) return false;
  return fut.get()->mode_sent;
}

// Kalıcı setpoint timer'ını garanti altına al
void MavrosClient::ensure_stream_(std::chrono::milliseconds dt) {
  if (!timer_ || dt != period_) {
    period_ = dt;
    timer_.reset();
    timer_ = node_->create_wall_timer(period_, [this]() {
      geometry_msgs::msg::PoseStamped sp;
      sp.header.stamp = node_->get_clock()->now();
      sp.header.frame_id = "map";
      sp.pose.orientation.w = 1.0;
      // x,y: varsa son pozisyonu koru; yoksa 0
      sp.pose.position.x = got_pose_ ? last_pose_.pose.position.x : 0.0;
      sp.pose.position.y = got_pose_ ? last_pose_.pose.position.y : 0.0;
      sp.pose.position.z = target_z_;
      sp_pub_->publish(sp);
    });
  }
}

void MavrosClient::pump_setpoints(double z, int count, std::chrono::milliseconds dt) {
  target_z_ = z;
  ensure_stream_(dt);

  // "Ön ısınma" için count kadar spin et -> yayın kesinlikle başlamış olsun
  for (int i = 0; i < count && rclcpp::ok(); ++i) {
    exec_->spin_some();
    rclcpp::sleep_for(dt);
  }
  // Not: timer akışı DEVAM EDİYOR; OFFBOARD/ARM sırasında kesilmeyecek.
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
