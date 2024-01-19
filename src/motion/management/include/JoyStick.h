#pragma once

#include <core/misc/Buffer.h>
#include <core/types.h>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <utility>

namespace clear {

<<<<<<< HEAD
=======

>>>>>>> bbd32c6f4d1ad268f36378b3bb3a975da76236b3
class JoyStick {
public:
  JoyStick(rclcpp::Node::SharedPtr nodeHandle);

  vector3_t getLinearVelCmd();

  scalar_t getYawVelCmd();

  scalar_t getHeightCmd();

  bool isStance();

  bool isTrotting();

  bool eStop();

<<<<<<< HEAD
  bool isStart();

=======
>>>>>>> bbd32c6f4d1ad268f36378b3bb3a975da76236b3
private:
  void joy_cb(const std::shared_ptr<sensor_msgs::msg::Joy> joy_msg) const;

  rclcpp::Node::SharedPtr nodeHandle_;

  Buffer<bool> e_stop_;
<<<<<<< HEAD
  Buffer<bool> start_;
=======
>>>>>>> bbd32c6f4d1ad268f36378b3bb3a975da76236b3
  mutable Buffer<std::shared_ptr<sensor_msgs::msg::Joy>> joy_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  scalar_t h_des_ = 0.0;
  vector3_t vel_cmd;
<<<<<<< HEAD
=======

>>>>>>> bbd32c6f4d1ad268f36378b3bb3a975da76236b3
};

} // namespace clear
