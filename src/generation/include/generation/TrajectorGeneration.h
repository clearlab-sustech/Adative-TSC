#pragma once

#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trans/msg/estimated_states.hpp>
#include <trans/msg/mode_schedule_trans.hpp>
#include <trans/msg/trajectory_array.hpp>
#include <tsc/tsc.h>

using namespace rclcpp;
using namespace std;

namespace clear {
class TrajectorGeneration : public Node {

public:
  TrajectorGeneration(string config_yaml);

  ~TrajectorGeneration();

private:
  void inner_loop();

  void updatePinocchioInterface();

  void generate_base_traj();

  void generate_footholds();

  void generate_foot_traj();

  void extract_foot_switch_info();

  void publish_traj();

  void estimated_state_callback(
      const trans::msg::EstimatedStates::SharedPtr msg) const;

  void mode_schedule_callback(
      const trans::msg::ModeScheduleTrans::SharedPtr msg) const;

  geometry_msgs::msg::Vector3 vec3dToMsg(vector3_t &vec);

private:
  rclcpp::Subscription<trans::msg::EstimatedStates>::SharedPtr
      estimated_state_subscription_;
  rclcpp::Subscription<trans::msg::ModeScheduleTrans>::SharedPtr
      mode_schedule_subscription_;
  rclcpp::Publisher<trans::msg::TrajectoryArray>::SharedPtr
      trajectories_pub_ptr_;

  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;

  mutable Buffer<trans::msg::EstimatedStates::SharedPtr> estimated_state_buffer;
  mutable Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;
  std::string base_name;

  std::shared_ptr<trans::msg::Trajectory> base_pos_msg;
  std::shared_ptr<trans::msg::Trajectory> base_rpy_msg;
  std::map<std::string, std::shared_ptr<trans::msg::Trajectory>> foot_pos_msg;
  std::map<std::string, std::pair<scalar_t, vector3_t>> footholds;
  std::map<std::string, vector3_t> footholds_nominal_pos;
  std::map<std::string, std::vector<scalar_t>> foot_switch_phase_;

  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_start_;
  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_end_;
  quadruped::contact_flag_t contact_flag_;
};

} // namespace clear
