#pragma once

/* QP-based Controller */
/* Variables x=[\qacc, \torque, \force]: (18 + 12 + 3 * 4) */

#include "less/Task.h"
#include <core/misc/Buffer.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <pinocchio/PinocchioInterface.h>
#include <qpsolver/QpSolver.h>
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

  void eval(std::vector<bool> torch_flag = {});

  std::shared_ptr<const ActuatorCmds> getActuatorCmds();

private:
  void solve();

  Task getNewtonEulerEquation();

  Task getMaintainContactTask();

  Task getFrictionCone();

  Task getTorqueLimits();

  Task getFloatingBaseTask();

  Task getSwingTask();

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
  Buffer<std::shared_ptr<ActuatorCmds>> actuator_cmds_buffer_;
  std::vector<bool> torch_flag_;
  std::vector<std::string> actuated_joints_name, foot_names;
  std::shared_ptr<ActuatorCmds> cmds;

  std::shared_ptr<QpSolver> solver_ptr;
};
} // namespace clear
