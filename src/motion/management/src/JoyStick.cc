#include "JoyStick.h"

namespace clear {
using std::placeholders::_1;

JoyStick::JoyStick(rclcpp::Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  joy_sub_ = nodeHandle_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(), std::bind(&JoyStick::joy_cb, this, _1));

  e_stop_.push(false);

  RCLCPP_INFO(nodeHandle_->get_logger(), "JoyStick Initialized!");
}

void JoyStick::joy_cb(const sensor_msgs::msg::Joy::SharedPtr joy_msg) const {
  joy_msg_.push(joy_msg);
}

vector3_t JoyStick::getLinearVelCmd() {
  const scalar_t linear_velocity_factor = 0.3;
  auto msg = joy_msg_.get();
  if (msg == nullptr) {
    return vector3_t::Zero();
  } else {
    vector3_t vel_cmd;
    vel_cmd.z() = 0;
    vel_cmd.x() =
        linear_velocity_factor * msg->axes.at(1);
    vel_cmd.y() =
        linear_velocity_factor * msg->axes.at(0);
    return vel_cmd;
  }
}

scalar_t JoyStick::getYawVelCmd() {
  const scalar_t angular_velocity_factor = 0.3;
  auto msg = joy_msg_.get();
  if (msg == nullptr) {
    return 0.0;
  } else {
    return angular_velocity_factor * msg->axes.at(3);
  }
}

} // namespace clear
