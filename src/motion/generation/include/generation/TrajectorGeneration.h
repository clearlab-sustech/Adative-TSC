#pragma once

#include <asserts/trajectory/TrajectoriesArray.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;
using namespace std;

namespace clear {
class TrajectorGeneration {

public:
  TrajectorGeneration(Node::SharedPtr nodeHandle, string config_yaml);

  ~TrajectorGeneration();

  void update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

  std::shared_ptr<ocs2::PrimalSolution> get_mpc_sol();

  std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface> get_robot_interface();

  void setVelCmd(vector3_t vd, scalar_t yawd);

private:
  void inner_loop();

  vector_t get_rbd_state();

  void  set_reference();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
      robot_interface_ptr_;
  std::shared_ptr<ocs2::CentroidalModelRbdConversions> conversions_ptr_;
  std::shared_ptr<ocs2::legged_robot::GaitReceiver> gait_receiver_ptr_;
  std::shared_ptr<ocs2::RosReferenceManager> reference_manager_ptr_;
  std::shared_ptr<ocs2::SqpMpc> mpc_ptr_;

  Buffer<std::shared_ptr<ocs2::PrimalSolution>> mpc_sol_buffer;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer;
  Buffer<std::shared_ptr<vector_t>> qvel_ptr_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;

  vector3_t vel_cmd;
  scalar_t yawd_;
};

} // namespace clear
