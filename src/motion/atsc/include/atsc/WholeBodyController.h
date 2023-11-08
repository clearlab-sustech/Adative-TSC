#pragma once

#include "atsc/MatrixDB.h"
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/TrajectoriesArray.h>
#include <core/misc/Buffer.h>
#include <fstream>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_mpc/SystemObservation.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

namespace clear {

using namespace quadruped;

struct ActuatorCommands {
  vector_t Kp;
  vector_t Kd;
  vector_t pos;
  vector_t vel;
  vector_t torque;

  void setZero(size_t n) {
    Kp.setZero(n);
    Kd.setZero(n);
    pos.setZero(n);
    vel.setZero(n);
    torque.setZero(n);
  }
};

// Decision Variables: x = [\dot v^T, F^T, \tau^T]^T
class WholeBodyController {
public:
  WholeBodyController(Node::SharedPtr nodeHandle, const std::string config_yaml,
                      std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
                          robot_interface_ptr);

  ~WholeBodyController();

  void loadTasksSetting(bool verbose = true);

  void update_mpc_sol(std::shared_ptr<ocs2::PrimalSolution> mpc_sol);

  void update_state(const std::shared_ptr<vector_t> qpos_ptr,
                    const std::shared_ptr<vector_t> qvel_ptr);

  std::shared_ptr<ActuatorCommands> optimize();

private:
  void formulate();

  void updateContactJacobi();

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  MatrixDB formulateFloatingBaseEulerNewtonEqu();
  MatrixDB formulateTorqueLimitsTask();
  MatrixDB formulateMaintainContactTask();
  MatrixDB formulateFrictionConeTask();
  MatrixDB formulateMomentumTask();
  MatrixDB formulateSwingLegTask();
  MatrixDB formulateContactForceTask();

  vector3_t compute_euler_angle_err(const vector3_t &rpy_m,
                                    const vector3_t &rpy_d);

  void differential_inv_kin();

  vector_t get_rbd_state();

private:
  Node::SharedPtr nodeHandle_;
  Buffer<std::shared_ptr<TrajectoriesArray>> refTrajBuffer_;
  Buffer<std::shared_ptr<ocs2::PrimalSolution>> mpc_sol_buffer;
  size_t mode_;

  size_t numDecisionVars_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;

  std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
      robot_interface_ptr_;
  std::shared_ptr<ocs2::CentroidalModelRbdConversions> conversions_ptr_;

  vector_t mpcInput, xDot;

  std::string base_name, robot_name;
  std::vector<std::string> foot_names, actuated_joints_name;

  matrix_t Jc;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  matrix_t weightSwingLeg_, weightMomentum_;
  scalar_t weightContactForce_;
  matrix_t swingKp_, swingKd_;
  scalar_t frictionCoeff_{};

  MatrixDB weighedTask, constraints;
  scalar_t dt_ = 0.002;
  vector_t joint_acc_;
  std::shared_ptr<ActuatorCommands> actuator_commands_;

  std::fstream log_stream;
};
} // namespace clear