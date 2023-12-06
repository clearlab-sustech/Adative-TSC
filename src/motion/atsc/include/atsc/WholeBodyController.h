#pragma once

#include "atsc/ConstructVectorField.h"
#include "atsc/MatrixDB.h"
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/ReferenceBuffer.h>
#include <core/misc/Buffer.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>

using namespace rclcpp;

namespace clear {

using namespace legged_robot;

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
               std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr);

  ~WholeBodyController();

  void loadTasksSetting(bool verbose = false);

  void update_trajectory_reference(
      std::shared_ptr<ReferenceBuffer> referenceTrajectoriesPtr);

  void
  update_base_policy(const std::shared_ptr<ConstructVectorField::VectorFieldCoeffs> policy);

  std::shared_ptr<ActuatorCommands> optimize();

private:
  void formulate();

  void updateContactJacobi();

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  MatrixDB formulateFloatingBaseEulerNewtonEqu();
  MatrixDB formulateTorqueLimitsTask();
  MatrixDB formulateMaintainContactTask();
  MatrixDB formulateFrictionConeTask();
  MatrixDB formulateBaseTask();
  MatrixDB formulateSwingLegTask();
  MatrixDB formulateContactForceTask();

  vector3_t compute_euler_angle_err(const vector3_t &rpy_m,
                                    const vector3_t &rpy_d);

  void differential_inv_kin();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<ReferenceBuffer> refTrajBuffer_;
  Buffer<size_t> mode_;
  Buffer<std::shared_ptr<ConstructVectorField::VectorFieldCoeffs>> base_policy_;

  size_t numDecisionVars_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;

  std::string base_name, robot_name;
  std::vector<std::string> foot_names, actuated_joints_name;

  matrix_t Jc;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  matrix_t weightSwingLeg_, weightBase_, weightMomentum_;
  scalar_t weightContactForce_;
  matrix_t swingKp_, swingKd_, baseKp_, baseKd_, momentumKp_, momentumKd_;
  scalar_t frictionCoeff_{};

  MatrixDB weighedTask, constraints;
  scalar_t dt_ = 0.002;
  vector_t joint_acc_;
  std::shared_ptr<ActuatorCommands> actuator_commands_;

  // std::fstream log_stream;
};
} // namespace clear