/// \file
/// \brief Fingertip-position controller for the powerhouse finger.
///
/// Subscribes to Cartesian fingertip targets and joint state, solves inverse
/// kinematics in fingerlib, then runs joint-space PD to publish desired
/// joint torques.

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <filesystem>
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "fingerlib/kinematics.hpp"
#include "fingerlib/simple_pd.hpp"
#include "fingerlib/workspace_lookup.hpp"

static constexpr std::array<const char *, 3> ALL_JOINTS = {
  "mcp_splay",
  "mcp_flexion",
  "pip_flexion",
};

static constexpr int N_FULL = static_cast<int>(ALL_JOINTS.size());

class FingertipPositionControl : public rclcpp::Node
{
public:
  FingertipPositionControl()
  : Node("fingertip_position_control")
    , ctrl_(10.0, 0.5, 5.0)
  {
    std::string default_csv_path = "../fingerlib/workspace_samples.csv";
    try {
      default_csv_path = ament_index_cpp::get_package_share_directory("finger_control") +
        "/config/workspace_samples.csv";
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "Unable to resolve finger_control share path for default workspace CSV: %s",
        e.what());
    }

    declare_parameter("workspace_csv_path", default_csv_path);
    const std::string csv_path = get_parameter("workspace_csv_path").as_string();

    // Get workspace lookup
    try {
      workspace_lookup_ = std::make_unique<fingerlib::WorkspaceLookup>(csv_path);
      RCLCPP_INFO(get_logger(), "Loaded workspace CSV: %s", csv_path.c_str());
    } catch (const std::exception & e) {
      workspace_lookup_.reset();
      RCLCPP_WARN(get_logger(), "Failed to initialize workspace lookup from '%s': %s",
        csv_path.c_str(), e.what());
    }

    declare_parameter<std::vector<double>>("kp", std::vector<double>(N_FULL, 0.2));
    declare_parameter<std::vector<double>>("kd", std::vector<double>(N_FULL, 0.01));
    declare_parameter("tau_max", 8.0);
    declare_parameter("vel_alpha", 0.5);  // Increased from 0.3 for less aggressive smoothing
    declare_parameter("vel_deadband", 0.01);  // Ignore small velocities to reduce noise
    declare_parameter("publish_rate", 100.0);

    declare_parameter("branch", 1);
    declare_parameter("tip_offset_m", 0.03);
    declare_parameter("ik_max_iterations", 20);
    declare_parameter("ik_tolerance", 1.0e-6);
    declare_parameter("ik_damping", 1.0e-4);
    declare_parameter("ik_step_gain", 1.0);
    declare_parameter("ik_error_threshold", 0.01);  // 1 cm - consider target out-of-reach
    declare_parameter("ik_failure_damp_gain", 0.5);  // Reduce gains when target unreachable

    const std::vector<double> kp = get_parameter("kp").as_double_array();
    const std::vector<double> kd = get_parameter("kd").as_double_array();
    const double tau_max = get_parameter("tau_max").as_double();
    const double vel_alpha = get_parameter("vel_alpha").as_double();
    const double vel_deadband = get_parameter("vel_deadband").as_double();
    const double rate_hz = get_parameter("publish_rate").as_double();

    branch_ = get_parameter("branch").as_int();
    tip_offset_m_ = get_parameter("tip_offset_m").as_double();
    ik_error_threshold_ = get_parameter("ik_error_threshold").as_double();
    ik_failure_damp_gain_ = get_parameter("ik_failure_damp_gain").as_double();

    ik_options_.branch = branch_;
    ik_options_.tip_offset_m = tip_offset_m_;
    ik_options_.position_weight = 1.0;
    ik_options_.orientation_weight = 0.0;
    ik_options_.max_iterations = get_parameter("ik_max_iterations").as_int();
    ik_options_.tolerance = get_parameter("ik_tolerance").as_double();
    ik_options_.damping = get_parameter("ik_damping").as_double();
    ik_options_.step_gain = get_parameter("ik_step_gain").as_double();

    vel_alpha_ = std::clamp(vel_alpha, 0.0, 1.0);
    vel_deadband_ = std::max(vel_deadband, 0.0);
    ctrl_ = fingerlib::PDController<N_FULL>(kp, kd, tau_max);

    torque_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/finger/desired_joint_torques", 10);

    target_state_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/finger/gui_target_feedback",
      rclcpp::QoS(1).reliable().transient_local());

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FingertipPositionControl::state_callback, this, std::placeholders::_1));

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/finger/fingertip_target", 10,
      std::bind(&FingertipPositionControl::target_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(period, std::bind(&FingertipPositionControl::control_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "fingertip_position_control ready | kp=[%.3f %.3f %.3f] kd=[%.3f %.3f %.3f] "
      "tau_max=%.2f branch=%d tip_offset=%.3f rate=%.1fHz vel_alpha=%.2f vel_deadband=%.3f",
      kp[0], kp[1], kp[2], kd[0], kd[1], kd[2], tau_max, branch_, tip_offset_m_, rate_hz, vel_alpha,
      vel_deadband);
  }

private:
  fingerlib::PDController<N_FULL> ctrl_;
  fingerlib::FingertipTrackingOptions ik_options_;
  std::unique_ptr<fingerlib::WorkspaceLookup> workspace_lookup_;

  bool state_received_{false};
  bool target_received_{false};
  bool dq_initialized_{false};
  bool q_des_initialized_{false};
  bool target_initialized_from_state_{false};

  int branch_{+1};
  double tip_offset_m_{0.03};
  double vel_alpha_{0.3};

  Eigen::Vector3d q_measured_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dq_measured_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d q_des_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();

  // Out-of-workspace detection
  double ik_error_threshold_{0.01};  // Consider unreachable if position error > this [m]
  double ik_failure_damp_gain_{0.5};  // Reduce gains when unreachable (unused when lookup enabled)

  // Velocity deadband
  double vel_deadband_{0.01};  // Ignore velocity magnitude below this [rad/s]

  // URDF limits for actuated joints.
  const Eigen::Vector3d q_min_{-0.174533, -0.785, 0.0};
  const Eigen::Vector3d q_max_{+0.174533, +0.785, 1.57};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_target_state()
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.point.x = target_position_.x();
    msg.point.y = target_position_.y();
    msg.point.z = target_position_.z();
    target_state_pub_->publish(msg);
  }

  static Eigen::Vector3d clamp_joints(
    const Eigen::Vector3d & q,
    const Eigen::Vector3d & q_min,
    const Eigen::Vector3d & q_max)
  {
    Eigen::Vector3d q_clamped = q;
    for (int i = 0; i < 3; ++i) {
      q_clamped[i] = std::clamp(q_clamped[i], q_min[i], q_max[i]);
    }
    return q_clamped;
  }

  bool extract_joint(
    const sensor_msgs::msg::JointState & msg,
    const std::string & name,
    double & position,
    double * velocity = nullptr) const
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), name);
    if (it == msg.name.end()) {
      return false;
    }

    const size_t idx = std::distance(msg.name.begin(), it);
    position = msg.position.size() > idx ? msg.position[idx] : 0.0;
    if (velocity) {
      *velocity = msg.velocity.size() > idx ? msg.velocity[idx] : 0.0;
    }
    return true;
  }

  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool ok = true;
    Eigen::Vector3d dq_raw = Eigen::Vector3d::Zero();
    for (int i = 0; i < N_FULL; ++i) {
      ok &= extract_joint(*msg, ALL_JOINTS[i], q_measured_[i], &dq_raw[i]);
    }

    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "One or more joints missing from /joint_states");
      return;
    }

    // Apply velocity deadband to reduce noise amplification in derivative term
    for (int i = 0; i < N_FULL; ++i) {
      if (std::abs(dq_raw[i]) < vel_deadband_) {
        dq_raw[i] = 0.0;
      }
    }

    if (!dq_initialized_) {
      dq_measured_ = dq_raw;
      dq_initialized_ = true;
    } else {
      dq_measured_ = vel_alpha_ * dq_raw + (1.0 - vel_alpha_) * dq_measured_;
    }

    if (!q_des_initialized_) {
      q_des_ = q_measured_;
      q_des_initialized_ = true;
    }

    if (!target_initialized_from_state_) {
      // Seed desired Cartesian target from measured startup pose to avoid initial jumps.
      target_position_ = fingerlib::fingertip_pose(q_measured_, tip_offset_m_,
        branch_).translation();
      target_received_ = true;
      target_initialized_from_state_ = true;
      publish_target_state();
      RCLCPP_INFO(
        get_logger(),
        "Initialized fingertip target from initial state: [%.4f, %.4f, %.4f] m",
        target_position_.x(), target_position_.y(), target_position_.z());
    }

    state_received_ = true;
  }

  void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    target_position_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    target_received_ = true;
    publish_target_state();
  }

  void control_loop()
  {
    if (!state_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /joint_states ...");
      return;
    }

    if (!target_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /finger/fingertip_target ...");
      return;
    }

    const auto current_pose = fingerlib::fingertip_pose(q_measured_, tip_offset_m_, branch_);
    Eigen::Isometry3d desired_pose = current_pose;
    desired_pose.translation() = target_position_;

    const Eigen::Vector3d q_guess = q_des_initialized_ ? q_des_ : q_measured_;
    Eigen::Vector3d q_des_unclamped = fingerlib::fingertip_inverse_kinematics(desired_pose, q_guess,
      ik_options_);
    Eigen::Vector3d q_des_clamped = clamp_joints(q_des_unclamped, q_min_, q_max_);
    q_des_ = q_des_clamped;
    // Detect if clamping occurred (joints hitting limits)
    bool joints_clamped = (q_des_unclamped - q_des_clamped).norm() > 1e-6;
    if (joints_clamped) {
      std::ostringstream unclamped_stream;
      std::ostringstream clamped_stream;
      unclamped_stream << "[" << q_des_unclamped.x() << ", " << q_des_unclamped.y() << ", "
                       << q_des_unclamped.z() << "]";
      clamped_stream << "[" << q_des_clamped.x() << ", " << q_des_clamped.y() << ", "
                     << q_des_clamped.z() << "]";

      RCLCPP_INFO(
        get_logger(),
        "Desired joint target was clamped | original(rad)=%s clamped(rad)=%s",
        unclamped_stream.str().c_str(), clamped_stream.str().c_str());
    }

    // If joints are clamped and we have a workspace lookup, prefer the nearest
    // sampled joint configuration instead of the unclamped IK solution.
    if (joints_clamped && workspace_lookup_) {
      RCLCPP_INFO(
      get_logger(),
      "Out of workspace");
      try {
        // Convert unclamped radians to degrees for lookup
        Eigen::Vector3d q_deg_unclamped(
          fingerlib::rad2deg(q_des_unclamped.x()),
          fingerlib::rad2deg(q_des_unclamped.y()),
          fingerlib::rad2deg(q_des_unclamped.z()));

        const auto & sample = workspace_lookup_->sample_for_joint_angles_deg(q_deg_unclamped);

        // Convert sampled degrees back to radians and clamp to limits
        const Eigen::Vector3d q_sample_rad(
          fingerlib::deg2rad(sample.q_deg.x()),
          fingerlib::deg2rad(sample.q_deg.y()),
          fingerlib::deg2rad(sample.q_deg.z()));

        q_des_ = clamp_joints(q_sample_rad, q_min_, q_max_);
        // Update the target position to the achievable sampled position
        // target_position_ = sample.position_m;
        publish_target_state();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Using workspace lookup sample for clamped target.");
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Workspace lookup failed: %s", e.what());
      }
    }

    // RCLCPP_INFO(
    //   get_logger(),
    //   "Desired joint target from fingertip IK (rad) (post lookup): [%.6f, %.6f, %.6f]",
    //   q_des_.x(), q_des_.y(), q_des_.z());

    // Compute achieved pose from desired joints (unused by lookup flow)
    // const auto achieved_pose = fingerlib::fingertip_pose(q_des_, tip_offset_m_, branch_);
    ctrl_.set_target(q_des_);
    const Eigen::Vector3d tau = ctrl_.compute(q_measured_, dq_measured_);

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(N_FULL);
    for (int i = 0; i < N_FULL; ++i) {
      cmd.data[i] = tau[i];
    }
    torque_pub_->publish(cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingertipPositionControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
