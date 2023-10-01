#pragma once

#include "generation/FootholdOptimization.h"
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/TrajectoriesArray.h>
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
  TrajectorGeneration(Node::SharedPtr nodeHandle, string config_yaml);

  ~TrajectorGeneration();

  void update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

  void update_mode_schedule(std::shared_ptr<ModeSchedule> mode_schedule);

  std::shared_ptr<const TrajectoriesArray> get_trajectory_reference();

  std::map<std::string, std::pair<scalar_t, vector3_t>> get_footholds();

  void setVelCmd(vector3_t vd, scalar_t yawd);

private:
  void inner_loop();

  void generate_base_traj(scalar_t t_now);

  void generate_footholds(scalar_t t_now);

  void generate_foot_traj(scalar_t t_now);

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<TrajectoriesArray> refTrajBuffer_;

  std::shared_ptr<FootholdOptimization> footholdOpt_ptr;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer;
  Buffer<std::shared_ptr<vector_t>> qvel_ptr_buffer;
  Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;
  std::string base_name;
  std::vector<std::string> foot_names;

  std::map<std::string, std::pair<scalar_t, vector3_t>> footholds;
  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_start_;
  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_end_;
  quadruped::contact_flag_t contact_flag_;

  vector3_t vel_cmd;
  scalar_t yawd_;
};

} // namespace clear
