/// \file
/// \brief Node for setting the DIP angle value according to the four bar linkage.
///        Note that this function is only used in rviz with GUI display, implemented
///        controllers deal with this constraint independently.
///
/// PARAMETERS:
///     branch (int): Four-bar assembly mode selector: +1 or -1. Default: +1.
/// PUBLISHES:
///     joint_states/ (sensor_msgs::msg::JointState): Full joint state vector with DIP angle corrected by four-bar closure.
/// SUBSCRIBES:
///     joint_states_raw/ (sensor_msgs::msg::JointState): Raw joint state vector with incorrect DIP angle value.
/// SERVERS:
///
/// CLIENTS:
///
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>

#include "fingerlib/kinematics.hpp"

/// \brief Node that corrects the DIP joint angle using the four-bar linkage closure equation
class FourBar : public rclcpp::Node
{
public:
  /// \brief FingerFourBar constructor
  FourBar()
  : Node("four_bar")
  {
    declare_parameter("branch", 1);
    branch_ = get_parameter("branch").as_int();

    // Publishers
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    joint_states_raw_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states_raw", 10,
      std::bind(&FourBar::joint_states_callback, this, std::placeholders::_1)
    );
  }

private:
  int branch_;                                                                                   // Four-bar assembly mode
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_raw_sub_;           // Raw joint state subscriber
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;                  // Corrected joint state publisher

  //
  // Subscriber callbacks
  //
  // Publish corrected joint state at same frequency as raw joint states
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Find PIP index in joint vector
    auto it = std::find(msg->name.begin(), msg->name.end(), "pip_flexion");
    if (it == msg->name.end()) {
      RCLCPP_WARN(get_logger(), "pip_flexion joint not found in joint_states_raw");
      return;
    }
    const size_t pip_idx = std::distance(msg->name.begin(), it);
    const auto pip_deg = msg->position[pip_idx] * (180.0 / M_PI);

    // Solve for dip angle and publish
    const auto dip_rad = fingerlib::solve_dip_from_pip(pip_deg, branch_);
    auto out = *msg;
    auto dip_it = std::find(out.name.begin(), out.name.end(), "dip_flexion");
    if (dip_it == out.name.end()) {
      RCLCPP_WARN(get_logger(), "dip_flexion joint not found in joint_states_raw");
      return;
    }

    const size_t dip_idx = std::distance(out.name.begin(), dip_it);
    out.position[dip_idx] = dip_rad;

    RCLCPP_DEBUG(get_logger(), "pip: %.4f rad  ->  dip: %.4f rad",
      msg->position[pip_idx], dip_rad);

    joint_states_pub_->publish(out);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FourBar>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
