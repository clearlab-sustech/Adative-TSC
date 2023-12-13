#pragma once

#include "control/ConstructVectorField.h"
#include "control/MatrixDB.h"
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/TrajectoriesArray.h>
#include <core/misc/Buffer.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>

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
                      const std::string config_yaml);

  ~WholeBodyController();

  void loadTasksSetting(bool verbose = true);

  void update_trajectory_reference(
      std::shared_ptr<TrajectoriesArray> referenceTrajectoriesPtr);

  void update_mode(size_t mode);

  void update_state(const std::shared_ptr<vector_t> qpos_ptr,
                    const std::shared_ptr<vector_t> qvel_ptr);

  void
  update_vector_field(const std::shared_ptr<ConstructVectorField::VectorFieldParam> vf);

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
  Buffer<std::shared_ptr<TrajectoriesArray>> refTrajBuffer_;
  Buffer<size_t> mode_;
  Buffer<std::shared_ptr<ConstructVectorField::VectorFieldParam>> base_vector_field_;

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

  std::fstream log_stream;
};
} // namespace clear