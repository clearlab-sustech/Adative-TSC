#pragma once

#include <core/misc/Buffer.h>
#include <core/types.h>
#include <map>
#include <mros/controller_msgs/IMUData.h>
#include <mros/controller_msgs/RobotCmdPointFoot.h>
#include <mros/controller_msgs/RobotStatePointFoot.h>
#include <mros/mros.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trans/msg/actuator_cmds.hpp>
#include <trans/msg/touch_sensor.hpp>
#include <vector>

using namespace std;
using namespace rclcpp;

namespace clear {

class MrosHW {
public:
  std::map<std::string, size_t> jointsName2IndexMap{};

  std::map<size_t, std::string> jointsIndex2NameMap{};

  MrosHW(Node::SharedPtr node_handle);

  ~MrosHW();

  sensor_msgs::msg::Imu::SharedPtr get_imu_msg();

  trans::msg::TouchSensor::SharedPtr get_touch_msg();

  sensor_msgs::msg::JointState::SharedPtr get_joint_msg();

  void set_actuator_cmds(const trans::msg::ActuatorCmds::SharedPtr msg);

  void read();

  void send();

  void switch_to_damping();

private:
  void init();

  Node::SharedPtr nodeHandle_;
  std::string robot_type_;


  sensor_msgs::msg::JointState::SharedPtr joints_state_ptr;
  sensor_msgs::msg::Imu::SharedPtr imu_data_ptr;
  trans::msg::TouchSensor::SharedPtr touch_sensor_ptr;

  Buffer<trans::msg::ActuatorCmds::SharedPtr> actuator_cmd_msg_buffer;

  mros::Subscriber<mros::controller_msgs::RobotStatePointFoot> *robotStateSub_{
      nullptr};
  mros::Subscriber<mros::controller_msgs::IMUData> *robotImuSub_ = {nullptr};
  mros::Publisher *robotCmdPub_ = {nullptr};

  std::atomic<mros::controller_msgs::RobotStatePointFoot *> atomicRobotState_{
      nullptr};
  std::atomic<mros::controller_msgs::RobotCmdPointFoot *> atomicRobotCmd_{
      nullptr};
  std::atomic<mros::controller_msgs::IMUData *> atomicRobotImu_{nullptr};

  std::atomic<int8_t> atomicRobotStateOn_{0};
  std::atomic<int8_t> atomicRobotCmdOn_{0};
  std::atomic<int8_t> atomicRobotImuOn_{0};

  std::vector<scalar_t> jointLimits_;   // NOLINT(modernize-avoid-c-arrays)
  std::vector<scalar_t> jointOffsets_;  // NOLINT(modernize-avoid-c-arrays)

  // Used for temporarily cache data from atomic operations
  mros::controller_msgs::IMUData robotImu_;
  mros::controller_msgs::RobotStatePointFoot robotState_;
  mros::controller_msgs::RobotCmdPointFoot robotCmd_;
};

} // namespace clear
