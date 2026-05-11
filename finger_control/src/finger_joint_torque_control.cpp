/// \file
/// \brief Converts desired joint torques into motor torque commands using
///        tendon tensions from fingerlib::tendon_tensions_soft_constraint.
///
/// PIPELINE:
///   desired joint torques -> tendon tensions -> motor torques
///
/// PUBLISHES:
///   /finger/motor_torque_commands (std_msgs::msg::Float64MultiArray)
///
/// SUBSCRIBES:
///   /finger/torque_commands (std_msgs::msg::Float64MultiArray)
///   /joint_states (sensor_msgs::msg::JointState)

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Core>

#include <exception>
#include <algorithm>
#include <iterator>
#include <memory>
#include <string>

#include "fingerlib/constants.hpp"
#include "fingerlib/kinematics.hpp"
#include "fingerlib/jacobian_lookup.hpp"

class FingerJointTorqueControl : public rclcpp::Node {
public:
  FingerJointTorqueControl()
  : Node("finger_joint_torque_control")
  {
    // Get Jacobain lookup CSV path
    std::string default_csv_path = "../fingerlib/jacobian_transposes.csv";
    try {
      default_csv_path = ament_index_cpp::get_package_share_directory("finger_control") +
        "/config/jacobian_transposes.csv";
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "Unable to resolve finger_control share path for default Jacobian CSV: %s",
        e.what());
    }

    declare_parameter("pulley_radius", 5.0);
    declare_parameter("jacobian_csv_path", default_csv_path);

    pulley_radius_ = get_parameter("pulley_radius").as_double();
    const std::string csv_path = get_parameter("jacobian_csv_path").as_string();

    try {
      jacobian_lookup_ = std::make_unique<fingerlib::JacobianLookup>(csv_path);
    } catch (const std::exception & e) {
      jacobian_lookup_.reset();
      RCLCPP_WARN(get_logger(),
        "Failed to initialize Jacobian lookup from '%s': %s. Using static Jacobian fallback.",
        csv_path.c_str(), e.what());
    }

    // Publishers
    motor_torque_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/finger/motor_torque_commands", 10);

    // Subscribers
    desired_joint_torque_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/finger/torque_commands", 10,
      std::bind(&FingerJointTorqueControl::desired_joint_torque_callback, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FingerJointTorqueControl::joint_state_callback, this, std::placeholders::_1));

    J_ << 1.0, 0.0, 0.0, 0.5,
      0.0, 1.0, 0.0, 0.5,
      0.0, 0.0, 1.0, 0.0;

    RCLCPP_INFO(get_logger(),
      "finger_joint_torque_control ready | pulley_radius=%.4f | jacobian_lookup=%s",
      pulley_radius_, jacobian_lookup_ ? "enabled" : "fallback_static");
  }

private:
  Eigen::Matrix<double, 3, 4> J_;         
  std::unique_ptr<fingerlib::JacobianLookup> jacobian_lookup_;
  double pulley_radius_{1.0};
  double pip_angle_deg_{0.0};
  bool has_joint_state_{false};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_torque_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_joint_torque_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Subscribe to joint states and use current PIP angle to select Jacobian
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const auto it = std::find(msg->name.begin(), msg->name.end(), "pip_flexion");
    if (it == msg->name.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "pip_flexion not found in /joint_states");
      return;
    }

    const std::size_t idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
    if (msg->position.size() <= idx) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "pip_flexion position missing in /joint_states");
      return;
    }

    pip_angle_deg_ = fingerlib::rad2deg(msg->position[idx]);
    has_joint_state_ = true;
  }

  // Compute motor torques from desired joint torques and publish
  void desired_joint_torque_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    Eigen::Matrix<double, 3, 1> desired_joint_torques = Eigen::Matrix<double, 3, 1>::Zero();

    for (int i = 0; i < 3 && i < static_cast<int>(msg->data.size()); ++i) {
      desired_joint_torques[i] = msg->data[i];
    }

    if (jacobian_lookup_ && has_joint_state_) {
      J_ = jacobian_lookup_->jacobian_for_angle_deg(static_cast<float>(pip_angle_deg_));
    }

    const Eigen::VectorXd tau_dynamic = desired_joint_torques.cast<double>();
    const Eigen::MatrixXd J_dynamic = J_.cast<double>();
    const Eigen::VectorXd tendon_tensions =
      fingerlib::tendon_tensions(tau_dynamic, J_dynamic);

    const Eigen::Matrix<double, 4, 1> motor_torques = tendon_tensions.head<4>() * pulley_radius_;

    std_msgs::msg::Float64MultiArray out;
    out.data = {motor_torques[0], motor_torques[1], motor_torques[2], motor_torques[3]};
    motor_torque_pub_->publish(out);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerJointTorqueControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}