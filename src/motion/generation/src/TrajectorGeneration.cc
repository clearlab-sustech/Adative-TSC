#include "generation/TrajectorGeneration.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

TrajectorGeneration::TrajectorGeneration(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();

  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  freq_ = config_["generation"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(rclcpp::get_logger("TrajectorGeneration"), "frequency: %f",
              freq_);

  refTrajBuffer_ = std::make_shared<ReferenceBuffer>();

  base_planner_ptr = std::make_shared<FloatingBaseMotion>(
      nodeHandle_, pinocchioInterface_ptr_, refTrajBuffer_);

  foothold_opt_ptr = std::make_shared<FootholdOptimization>(
      nodeHandle_, pinocchioInterface_ptr_, refTrajBuffer_);

  swing_traj_ptr = std::make_shared<SwingTrajectory>(
      nodeHandle_, pinocchioInterface_ptr_, refTrajBuffer_);

  run_.push(true);
  inner_loop_thread_ = std::thread(&TrajectorGeneration::inner_loop, this);
}

TrajectorGeneration::~TrajectorGeneration() {
  run_.push(false);
  inner_loop_thread_.join();
}

void TrajectorGeneration::update_current_state(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectorGeneration::update_mode_schedule(
    std::shared_ptr<ModeSchedule> mode_schedule) {
  refTrajBuffer_.get()->set_mode_schedule(mode_schedule);
}

std::shared_ptr<ReferenceBuffer>
TrajectorGeneration::get_trajectory_reference() {
  return refTrajBuffer_;
}

void TrajectorGeneration::inner_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();

    if (qpos_ptr_buffer.get() != nullptr && qvel_ptr_buffer.get() != nullptr) {

      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

      foothold_opt_ptr->optimize();

      base_planner_ptr->generate();

      swing_traj_ptr->generate();
    }

    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectorGeneration"),
              "max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void TrajectorGeneration::setVelCmd(vector3_t vd, scalar_t yawd) {
  foothold_opt_ptr->setVelCmd(vd, yawd);
}

} // namespace clear
