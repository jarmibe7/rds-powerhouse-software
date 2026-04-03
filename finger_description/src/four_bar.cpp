/// \file
/// \brief Node for setting the DIP angle value according to the four bar linkage
///
/// PARAMETERS:
///     
/// PUBLISHES:
///     joint_states/ (sensor_msgs::msg::JointState): Publishes the full joint state vector, with the DIP angle value corrected according to the four bar linkage
/// SUBSCRIBES:
///     joint_states_raw/ (sensor_msgs::msg::JointState): The raw joint state vector, with incorrect DIP angle value
/// SERVERS:
///     
/// CLIENTS:
///
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
 

template<typename T>
T dipAngleFromPip(T pip_angle)
{
    return static_cast<T>(0.85) * pip_angle;
}
 
// ---------------------------------------------------------------------------
class FingerFourBar : public rclcpp::Node
{
public:
    FingerFourBar() : Node("FingerFourBar")
    {
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
 
                auto it = std::find(msg->name.begin(), msg->name.end(), "pip_flexion");
                if (it == msg->name.end()) return;
 
                size_t idx = std::distance(msg->name.begin(), it);
                double dip_angle = dipAngleFromPip(msg->position[idx]);
 
                RCLCPP_INFO(this->get_logger(), "pip: %.4f  ->  dip: %.4f",
                    msg->position[idx], dip_angle);
            });
    }
 
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};
 
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FingerFourBar>());
    rclcpp::shutdown();
    return 0;
}