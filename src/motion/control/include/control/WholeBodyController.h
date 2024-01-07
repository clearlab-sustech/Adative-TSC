#pragma once

#include "control/ConstructVectorField.h"
#include "control/MatrixDB.h"
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <fstream>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
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
  WholeBodyController(Node::SharedPtr nodeHandle,
                      std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
                          robot_interface_ptr);

  ~WholeBodyController();

  void loadTasksSetting(bool verbose = true);

  void update_mpc_sol(std::shared_ptr<ocs2::PrimalSolution> mpc_sol);

  void update_state(const std::shared_ptr<vector_t> qpos_ptr,
                    const std::shared_ptr<vector_t> qvel_ptr);

  void updateBaseVectorField(
      const std::shared_ptr<ConstructVectorField::VectorFieldParam> vf);

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
  MatrixDB formulateJointTask();
  MatrixDB formulateContactForceTask();

  void differential_inv_kin();

  vector_t get_rbd_state();

private:
  Node::SharedPtr nodeHandle_;
  Buffer<std::shared_ptr<ocs2::PrimalSolution>> mpc_sol_buffer;
  size_t mode_;

  size_t numDecisionVars_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;

  std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
      robot_interface_ptr_;
  std::shared_ptr<ocs2::CentroidalModelRbdConversions> conversions_ptr_;
  Buffer<std::shared_ptr<ConstructVectorField::VectorFieldParam>> base_vf_;

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
  scalar_t t0 = 0.0;

  std::fstream log_stream;
};
} // namespace clear