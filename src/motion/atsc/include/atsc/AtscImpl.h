#pragma once
#include "atsc/WholeBodyController.h"
#include <core/misc/Buffer.h>
#include <memory>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trans/msg/actuator_cmds.hpp>

using namespace rclcpp;

namespace clear {
class AtscImpl {

public:
  AtscImpl(Node::SharedPtr nodeHandle, const std::string config_yaml,
           std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
               robot_interface_ptr);

  ~AtscImpl();

  void update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

  void update_mpc_solution(std::shared_ptr<ocs2::PrimalSolution> mpc_sol);

  trans::msg::ActuatorCmds::SharedPtr getCmds();

private:
  void publishCmds();

  void inner_loop();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
      robot_interface_ptr_;

  rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
      actuators_cmds_pub_ptr_;

  Buffer<std::shared_ptr<ocs2::PrimalSolution>> mpc_sol_buffer;
  std::shared_ptr<WholeBodyController> wbcPtr_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;

  std::string base_name, robot_name;
  std::vector<std::string> actuated_joints_name;
  std::shared_ptr<ActuatorCommands> actuator_commands_;
};

} // namespace clear
