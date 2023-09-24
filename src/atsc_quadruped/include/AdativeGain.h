#pragma once
#include <core/misc/Buffer.h>
#include <core/trajectory/PoseTrajectory.h>
#include <description/ModelInfo.h>
#include <pinocchio/PinocchioInterface.h>

#include <hpipm-cpp/hpipm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/ControlData.h"
#include "common/ModeSchedule.h"
#include "common/MotionPhaseDefinition.h"
#include "common/TrajectoriesBuffer.h"

using namespace rclcpp;

namespace clear {
class ContactForceOptimization {
public:
  ContactForceOptimization(
      Node::SharedPtr nodeHandle, std::shared_ptr<ModelInfo> modelInfoPtr,
      std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr);

  ~ContactForceOptimization();

  void update_trajectory_reference(
      std::shared_ptr<const TrajectoriesBuffer> referenceTrajectoriesPtr);

  void update_mode_schedule(const std::shared_ptr<ModeSchedule> mode_schedule);

  void update_state(const std::shared_ptr<vector_t> qpos_ptr,
                    const std::shared_ptr<vector_t> qvel_ptr);

  std::shared_ptr<CtrlData> optimize();

private:
  void step1(size_t k);

  void step2(size_t k, size_t N);

  void step3(size_t k, size_t N);

  vector3_t compute_euler_angle_err(const vector3_t &rpy_m,
                                    const vector3_t &rpy_d);

private:
  Node::SharedPtr nodeHandle_;
  const ModelInfo modelInfo_;
  PinocchioInterface pinocchioInterface_;
  PinocchioInterface pinocchioInterface_map_;

  Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;
  Buffer<std::shared_ptr<const TrajectoriesBuffer>>
      referenceTrajectoriesPtrBuffer_;
  std::shared_ptr<CtrlData> CtrlData_ptr;
  const scalar_t dt_ = 0.02;
  const scalar_t grav_ = 9.81;
  scalar_t total_mass_;
  matrix3_t Ig_;
  const scalar_t mu_ = 0.5;
  matrix_t weight_;

  std::vector<hpipm::OcpQp> ocp_;
  std::vector<hpipm::OcpQpSolution> solution_;
  hpipm::OcpQpIpmSolverSettings solver_settings;
  bool has_sol_ = false;
};

} // namespace clear
