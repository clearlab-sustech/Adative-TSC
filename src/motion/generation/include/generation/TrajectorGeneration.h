#pragma once

#include "generation/FloatingBaseMotion.h"
#include "generation/FootholdOptimization.h"
#include "generation/SwingTrajectory.h"
#include <asserts/trajectory/ReferenceBuffer.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;
using namespace std;

namespace clear {
class TrajectorGeneration {

public:
  TrajectorGeneration(Node::SharedPtr nodeHandle);

  ~TrajectorGeneration();

  void update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

  void update_mode_schedule(std::shared_ptr<ModeSchedule> mode_schedule);

  std::shared_ptr<ReferenceBuffer> get_trajectory_reference();

  void setVelCmd(vector3_t vd, scalar_t yawd);

private:
  void inner_loop();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> refTrajBuffer_;

  std::shared_ptr<FloatingBaseMotion> base_planner_ptr;
  std::shared_ptr<FootholdOptimization> foothold_opt_ptr;
  std::shared_ptr<SwingTrajectory> swing_traj_ptr;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer;
  Buffer<std::shared_ptr<vector_t>> qvel_ptr_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;

  vector3_t vel_cmd;
  scalar_t yawd_;
};

} // namespace clear
