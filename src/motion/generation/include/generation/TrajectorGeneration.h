#pragma once

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

private:
  void inner_loop();

  void generate_base_traj(scalar_t t_now);

  void generate_footholds(scalar_t t_now);

  void generate_foot_traj(scalar_t t_now);

  void extract_foot_switch_info();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<TrajectoriesArray> refTrajBuffer_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer;
  Buffer<std::shared_ptr<vector_t>> qvel_ptr_buffer;
  Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;
  std::string base_name;

  std::map<std::string, std::pair<scalar_t, vector3_t>> footholds;
  std::map<std::string, vector3_t> footholds_nominal_pos;
  std::map<std::string, std::vector<scalar_t>> foot_switch_phase_;

  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_start_;
  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_end_;
  quadruped::contact_flag_t contact_flag_;
};

} // namespace clear
