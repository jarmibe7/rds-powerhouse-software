/// \file
/// \brief Converts desired joint torques into motor torque commands using
///        a tendon-tension intermediary solved with NNLS.
///
/// PIPELINE:
///   desired joint torques -> tendon tensions -> motor torques
///
/// PUBLISHES:
///   /finger/motor_torque_commands (std_msgs::msg::Float64MultiArray)
///
/// SUBSCRIBES:
///   /finger/desired_joint_torques (std_msgs::msg::Float64MultiArray)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Core>
#include <array>
#include <algorithm>

#include "fingerlib/kinematics.hpp"

class FingerMotorControl : public rclcpp::Node {
public:
  FingerMotorControl()
  : Node("finger_motor_control")
  {
    declare_parameter("pulley_radius", 1.0);
    pulley_radius_ = get_parameter("pulley_radius").as_double();

    motor_torque_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/finger/motor_torque_commands", 10);

    desired_joint_torque_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/finger/desired_joint_torques", 10,
      std::bind(&FingerMotorControl::desired_joint_torque_callback, this, std::placeholders::_1));

    // Static 3x4 tendon Jacobian proxy.
    // Rows: [mcp_splay, mcp_flexion, pip_flexion]
    // Cols: [tendon_0, tendon_1, tendon_2, tendon_3]
    J_ << 1.0, 0.0, 0.0, 0.5,
          0.0, 1.0, 0.0, 0.5,
          0.0, 0.0, 1.0, 0.0;

    RCLCPP_INFO(get_logger(),
      "finger_motor_control ready | pulley_radius=%.4f", pulley_radius_);
  }

private:
  Eigen::Matrix<double, 3, 4> J_;
  double pulley_radius_{1.0};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_torque_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_joint_torque_sub_;

  void desired_joint_torque_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    Eigen::Matrix<double, 3, 1> desired_joint_torques = Eigen::Matrix<double, 3, 1>::Zero();

    // Load desired joint torques for the 3 controllable joints
    for (int i = 0; i < 3 && i < static_cast<int>(msg->data.size()); ++i) {
      desired_joint_torques[i] = msg->data[i];
    }

    // Solve NNLS for 4 tendon tensions from 3 desired joint torques
    // NNLS enforces nonnegative tensions 
    Eigen::MatrixXd J_dynamic = J_.cast<double>();
    Eigen::VectorXd tau_dynamic = desired_joint_torques.cast<double>();
    fingerlib::NNLS<Eigen::MatrixXd> nnls(J_dynamic);
    const Eigen::VectorXd tensions = nnls.solve(tau_dynamic);

    const Eigen::Matrix<double, 4, 1> motor_torques = tensions.head<4>() * pulley_radius_;

    std_msgs::msg::Float64MultiArray out;
    out.data = {motor_torques[0], motor_torques[1], motor_torques[2], motor_torques[3]};
    motor_torque_pub_->publish(out);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerMotorControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
