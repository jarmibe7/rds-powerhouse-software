/// \file
/// \brief Position PD control node for the powerhouse finger for testing and stuff
///
/// PARAMETERS:
///     kp            (double[]): Proportional gains for actuated joints, in order.
///                               Default: [0.1, 0.1, 0.1]
///     kd            (double[]): Derivative gains for actuated joints, in order.
///                               Default: [0.0025, 0.0025, 0.0025]
///     tau_max       (double): Torque clamp, actuated joints [N·m]. Default: 5.0
///     vel_alpha (double): Velocity LPF alpha in [0, 1]. Default: 0.2
///     branch        (int):    Four-bar branch selector (+1 or -1). Default: +1
///     publish_rate  (double): Control loop rate [Hz]. Default: 100.0
///
/// PUBLISHES:
///     /finger/desired_joint_torques (std_msgs::msg::Float64MultiArray):
///         Torques [N·m] in order: mcp_splay, mcp_flexion, pip_flexion, dip_flexion.
/// SUBSCRIBES:
///     /joint_states          (sensor_msgs::msg::JointState): Measured state from Drake / hardware.
///     /finger/joint_targets  (sensor_msgs::msg::JointState): Desired positions for the 3 actuated DOFs.
///
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "fingerlib/simple_pd.hpp"

// ── Joint ordering ────────────────────────────────────────────────────────────

// Full plant order published to Drake
static constexpr std::array<const char*, 3> ALL_JOINTS = {
  "mcp_splay",
  "mcp_flexion",
  "pip_flexion",
};

static constexpr int N_FULL = static_cast<int>(ALL_JOINTS.size());

// ── Node ─────────────────────────────────────────────────────────────────────

class FingerPDControl : public rclcpp::Node
{
public:
  FingerPDControl()
  : Node("finger_pd_control")
  , ctrl_(10.0, 0.5, 5.0)
  , state_received_(false)
  {
    // Parameters
    declare_parameter<std::vector<double>>("kp", std::vector<double>(N_FULL, 0.1));
    declare_parameter<std::vector<double>>("kd", std::vector<double>(N_FULL, 0.0025));
    declare_parameter("tau_max", 8.0);
    declare_parameter("vel_alpha", 0.3);
    declare_parameter("branch", 1);
    declare_parameter("publish_rate", 100.0);

    const std::vector<double> kp = get_parameter("kp").as_double_array();
    const std::vector<double> kd = get_parameter("kd").as_double_array();
    const double tau_max = get_parameter("tau_max").as_double();
    const double vel_alpha = get_parameter("vel_alpha").as_double();
    const double rate_hz = get_parameter("publish_rate").as_double();

    vel_alpha_ = std::clamp(vel_alpha, 0.0, 1.0);

    ctrl_ = fingerlib::PDController<N_FULL>(kp, kd, tau_max);

    // Publishers
    torque_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/finger/desired_joint_torques", 10
    );

    // Subscribers
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FingerPDControl::state_callback, this, std::placeholders::_1)
    );
    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/finger/joint_targets", 10,
      std::bind(&FingerPDControl::target_callback, this, std::placeholders::_1)
    );

    // Control timer
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      period, std::bind(&FingerPDControl::control_loop, this)
    );

    const auto kp_str = vec_to_string(kp);
    const auto kd_str = vec_to_string(kd);

    RCLCPP_INFO(get_logger(),
      "finger_pd_control ready | "
      "kp=%s kd=%s tau_max=%.2f vel_alpha=%.2f | "
      "rate=%.0f Hz",
      kp_str.c_str(), kd_str.c_str(), tau_max, vel_alpha_, rate_hz);
  }

private:
  fingerlib::PDController<N_FULL> ctrl_;
  bool   state_received_;
  bool   dq_initialized_{false};
  double vel_alpha_{0.2};

  // State storage
  Eigen::Matrix<double, N_FULL, 1> q_measured_ = Eigen::Matrix<double, N_FULL, 1>::Zero();
  Eigen::Matrix<double, N_FULL, 1> dq_measured_ = Eigen::Matrix<double, N_FULL, 1>::Zero();

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  //
  // Helpers
  //

  // Extract a single named joint's position and optional velocity from a JointState.
  // Returns false if the joint is not present.
  // Return by ref
  bool extract_joint(
    const sensor_msgs::msg::JointState & msg,
    const std::string & name,
    double & position,
    double * velocity = nullptr) const
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), name);
    if (it == msg.name.end()) return false;
    const size_t idx = std::distance(msg.name.begin(), it);
    position = msg.position.size() > idx ? msg.position[idx] : 0.0;
    if (velocity) {
      *velocity = msg.velocity.size() > idx ? msg.velocity[idx] : 0.0;
    }
    return true;
  }

  static std::string vec_to_string(const std::vector<double> & values)
  {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << values[i];
    }
    oss << "]";
    return oss.str();
  }

  // Callbacks
  // Receive sim encoder pos/vel data
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Read joint states and check for errors
    bool ok = true;
    Eigen::Matrix<double, N_FULL, 1> dq_raw = Eigen::Matrix<double, N_FULL, 1>::Zero();
    for (int i = 0; i < N_FULL; ++i) {
      ok &= extract_joint(*msg, ALL_JOINTS[i],
                          q_measured_[i], &dq_raw[i]);
    }

    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "One or more joints missing from /joint_states");
      return;
    }

    if (!dq_initialized_) {
      dq_measured_ = dq_raw;
      dq_initialized_ = true;
    } else {
      dq_measured_ = vel_alpha_ * dq_raw + (1.0 - vel_alpha_) * dq_measured_;
    }

    state_received_ = true;
  }

  // Receive target position data
  void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto ok = true;

    const auto has_vel = msg->velocity.size() >= msg->name.size();

    Eigen::Matrix<double, N_FULL, 1> q_des = Eigen::Matrix<double, N_FULL, 1>::Zero();
    Eigen::Matrix<double, N_FULL, 1> dq_des = Eigen::Matrix<double, N_FULL, 1>::Zero();

    // Read target joint positions and look for errors
    for (int i = 0; i < N_FULL; ++i) {
      double* vel_ptr = has_vel ? &dq_des[i] : nullptr;
      ok &= extract_joint(*msg, ALL_JOINTS[i], q_des[i], vel_ptr);
    }

    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "One or more actuated joints missing from /finger/joint_targets");
      return;
    }

    has_vel ? ctrl_.set_target(q_des, dq_des) : ctrl_.set_target(q_des);

    RCLCPP_DEBUG(get_logger(), "New target: [%.3f, %.3f, %.3f] rad",
      q_des[0], q_des[1], q_des[2]);
  }

  // Control loop
  void control_loop()
  {
    if (!state_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for /joint_states ...");
      return;
    }

    // PD for actuated joints
    const auto tau = ctrl_.compute(q_measured_, dq_measured_);

    // Pack into full 4-DOF command in Drake's joint order
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(N_FULL);
    for (int i = 0; i < N_FULL; ++i) {
      cmd.data[i] = tau[i];
    }

    torque_pub_->publish(cmd);

    RCLCPP_DEBUG(get_logger(),
      "tau: [%.3f, %.3f, %.3f] N·m",
      tau[0], tau[1], tau[2]);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerPDControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}