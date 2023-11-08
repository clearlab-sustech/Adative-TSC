#pragma once

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <asserts/trajectory/TrajectoriesArray.h>
#include <core/misc/Buffer.h>
#include <memory>
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

  std::shared_ptr<TrajectoriesArray> get_trajectory_reference();

  void setVelCmd(vector3_t vd, scalar_t yawd);

private:
  void inner_loop();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<TrajectoriesArray> refTrajBuffer_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer;
  Buffer<std::shared_ptr<vector_t>> qvel_ptr_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;

  vector3_t vel_cmd;
  scalar_t yawd_;
};

} // namespace clear
