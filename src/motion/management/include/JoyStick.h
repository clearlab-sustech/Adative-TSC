#pragma once

#include <core/misc/Buffer.h>
#include <core/types.h>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <utility>

namespace clear {

enum class ButtonIndex {
  LINEAR_VEL_Y = 0,
  LINEAR_VEL_X = 1,
  YAW_VEL = 3
};

class JoyStick {
public:
  JoyStick(rclcpp::Node::SharedPtr nodeHandle);

  vector3_t getLinearVelCmd();

  scalar_t getYawVelCmd();

private:
  void joy_cb(const std::shared_ptr<sensor_msgs::msg::Joy> joy_msg) const;

  rclcpp::Node::SharedPtr nodeHandle_;

  ButtonIndex button_index_;
  Buffer<bool> e_stop_;
  mutable Buffer<std::shared_ptr<sensor_msgs::msg::Joy>> joy_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

} // namespace clear
