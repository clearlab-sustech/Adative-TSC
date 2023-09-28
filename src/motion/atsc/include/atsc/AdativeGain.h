#pragma once
#include "asserts/gait/ModeSchedule.h"
#include "asserts/gait/MotionPhaseDefinition.h"
#include "asserts/trajectory/TrajectoriesArray.h"

#include <core/misc/Buffer.h>
#include <hpipm-cpp/hpipm-cpp.hpp>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

namespace clear {
class AdaptiveGain {
public:
  struct FeedbackGain {
    matrix_t K;
    vector_t b;
    vector_t force_des;
    vector_t state_des;
    vector_t state_dot_des;
  };

public:
  AdaptiveGain(Node::SharedPtr nodeHandle,
               std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr,
               std::string base_name);

  ~AdaptiveGain();

  void update_trajectory_reference(
      std::shared_ptr<const TrajectoriesArray> referenceTrajectoriesPtr);

  void update_mode_schedule(const std::shared_ptr<ModeSchedule> mode_schedule);

  std::shared_ptr<FeedbackGain> compute();

private:
  void add_linear_system(size_t k);

  void add_state_input_constraints(size_t k, size_t N);

  void add_cost(size_t k, size_t N);

  vector3_t compute_euler_angle_err(const vector3_t &rpy_m,
                                    const vector3_t &rpy_d);

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  std::string base_name_;

  Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;
  Buffer<std::shared_ptr<const TrajectoriesArray>> refTrajBuffer_;
  std::shared_ptr<FeedbackGain> feedback_law_ptr;
  const scalar_t dt_ = 0.02;
  const scalar_t grav_ = 9.81;
  scalar_t total_mass_;
  matrix3_t Ig_, Ig_inv;
  const scalar_t mu_ = 0.5;
  matrix_t weight_;

  std::vector<hpipm::OcpQp> ocp_;
  std::vector<hpipm::OcpQpSolution> solution_;
  hpipm::OcpQpIpmSolverSettings solver_settings;
  bool has_sol_ = false;
};

} // namespace clear
