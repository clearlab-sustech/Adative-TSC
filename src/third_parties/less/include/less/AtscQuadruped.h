#pragma once

/* QP-based Controller */
/* Variables x=[\qacc, \torque, \force]: (18 + 12 + 3 * 4) */

#include "less/FrictionCone.h"
#include "less/MaintainContactTask.h"
#include "less/NewtonEulerEquation.h"
#include "less/SE3Task.h"
#include "less/TorqueLimits.h"

#include <core/misc/Buffer.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

using namespace std;

namespace clear {
class AtscQuadruped {
public:
  AtscQuadruped(const std::string config_yaml, rclcpp::Logger logger);

  ~AtscQuadruped();

  struct ActuatorCmds {
    std::vector<std::string> names;
    vector_t Kp, Kd, pos, vel, tau;
  };

  void
  update_state(const nav_msgs::msg::Odometry &odom, // vel in local coordinate
               const sensor_msgs::msg::JointState &joints_state);

  void eval();

  std::shared_ptr<const ActuatorCmds> getActuatorCmds();

  shared_ptr<NewtonEulerEquation> newton_euler_eq_ptr;
  shared_ptr<MaintainContactTask> contact_ptr;
  shared_ptr<FrictionCone> friction_cone_ptr;
  shared_ptr<TorqueLimits> torque_limits_ptr;
  std::vector<std::shared_ptr<SE3Task>> se3_array_ptr;

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
  Buffer<std::shared_ptr<ActuatorCmds>> actuator_cmds_buffer_;
  std::vector<std::string> actuated_joints_name;
};
} // namespace clear
