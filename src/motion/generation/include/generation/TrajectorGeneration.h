#pragma once
#include <core/misc/Buffer.h>
#include <core/trajectory/ReferenceBuffer.h>
#include <core/types.h>
#include <memory>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;
using namespace std;

namespace clear {
class TrajectorGeneration {

public:
  TrajectorGeneration(Node::SharedPtr nodeHandle);

  ~TrajectorGeneration();

  void updateCurrentState(std::shared_ptr<vector_t> qpos_ptr,
                          std::shared_ptr<vector_t> qvel_ptr);

  std::shared_ptr<ocs2::PrimalSolution> getMpcSol();

  std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface> getRobotInterface();

  void setVelCmd(vector3_t vd, scalar_t yawd);

  std::shared_ptr<ReferenceBuffer> getReferenceBuffer();

private:
  void innerLoop();

  vector_t getRbdState();

  void setReference();

  void fitTraj();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
      robot_interface_ptr_;
  std::shared_ptr<ocs2::CentroidalModelRbdConversions> conversions_ptr_;
  std::shared_ptr<ocs2::legged_robot::GaitReceiver> gait_receiver_ptr_;
  std::shared_ptr<ocs2::RosReferenceManager> reference_manager_ptr_;
  std::shared_ptr<ocs2::SqpMpc> mpc_ptr_;

  Buffer<std::shared_ptr<ocs2::PrimalSolution>> mpc_sol_buffer;
  std::shared_ptr<ocs2::CentroidalModelPinocchioMapping> mapping_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer;
  Buffer<std::shared_ptr<vector_t>> qvel_ptr_buffer;

  std::shared_ptr<ReferenceBuffer> referenceBuffer_;

  std::string base_name;
  std::vector<string> foot_names;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;
  scalar_t t0 = 0.0;
  
  vector3_t vel_cmd;
  scalar_t yawd_;

  vector3_t pos_start;
  vector3_t rpy_zyx_start;
  bool first_run = true;
};

} // namespace clear
