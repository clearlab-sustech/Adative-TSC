#pragma once

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <core/misc/Buffer.h>
#include <core/types.h>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trans/msg/actuator_cmds.hpp>
#include <trans/msg/touch_sensor.hpp>
#include <vector>

using namespace std;
using namespace rclcpp;
using namespace UNITREE_LEGGED_SDK;

namespace clear {

class UnitreeHW : public Node {
public:
  std::map<std::string, size_t> jointsName2IndexMap{
      {"FL_hip_joint", FL_0},   {"FL_thigh_joint", FL_1},
      {"FL_calf_joint", FL_2},  {"FR_hip_joint", FR_0},
      {"FR_thigh_joint", FR_1}, {"FR_calf_joint", FR_2},
      {"RL_hip_joint", RL_0},   {"RL_thigh_joint", RL_1},
      {"RL_calf_joint", RL_2},  {"RR_hip_joint", RR_0},
      {"RR_thigh_joint", RR_1}, {"RR_calf_joint", RR_2}};

  std::map<size_t, std::string> jointsIndex2NameMap{
      {FL_0, "FL_hip_joint"},   {FL_1, "FL_thigh_joint"},
      {FL_2, "FL_calf_joint"},  {FR_0, "FR_hip_joint"},
      {FR_1, "FR_thigh_joint"}, {FR_2, "FR_calf_joint"},
      {RL_0, "RL_hip_joint"},   {RL_1, "RL_thigh_joint"},
      {RL_2, "RL_calf_joint"},  {RR_0, "RR_hip_joint"},
      {RR_1, "RR_thigh_joint"}, {RR_2, "RR_calf_joint"}};

  UnitreeHW(const std::string config_yaml);

  ~UnitreeHW();

  bool init();

  void updateJoystick();

private:
  void inner_loop();

  void imu_callback();

  void touch_callback();

  void joint_callback();

  void
  actuator_cmd_callback(const trans::msg::ActuatorCmds::SharedPtr msg) const;

  void read();

  void write();

  void drop_old_message();

  std::string robot_type_;

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState lowState_{};
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};

  int powerLimit_ = 4;
  int contactThreshold_ = 40;

  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<trans::msg::TouchSensor>::SharedPtr touch_publisher_;
  rclcpp::Subscription<trans::msg::ActuatorCmds>::SharedPtr
      actuator_cmd_subscription_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;

  Buffer<sensor_msgs::msg::Imu::SharedPtr> imu_msg_buffer;
  Buffer<trans::msg::TouchSensor::SharedPtr> touch_msg_buffer;
  Buffer<sensor_msgs::msg::JointState::SharedPtr> joint_state_msg_buffer;
  mutable Buffer<trans::msg::ActuatorCmds::SharedPtr> actuator_cmd_msg_buffer;
};

} // namespace clear
