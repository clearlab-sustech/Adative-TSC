#include <MotionManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

MotionManager::MotionManager(std::string config_yaml)
    : Node("MotionManager"), config_yaml_(config_yaml) {}

MotionManager::~MotionManager() {
  run_.push(false);
  inner_loop_thread_.join();
}

void MotionManager::init() {
  estimatorPtr_ = std::make_shared<StateEstimationLKF>(this->shared_from_this(),
                                                       config_yaml_);
  gaitSchedulePtr_ =
      std::make_shared<GaitSchedule>(this->shared_from_this(), config_yaml_);
  trajGenPtr_ = std::make_shared<TrajectorGeneration>(this->shared_from_this(),
                                                      config_yaml_);
  atscImplPtr_ =
      std::make_shared<AtscImpl>(this->shared_from_this(), config_yaml_);

  visPtr_ = std::make_shared<DataVisualization>(this->shared_from_this(),
                                                config_yaml_);

  inner_loop_thread_ = std::thread(&MotionManager::inner_loop, this);
  run_.push(true);
}

void MotionManager::inner_loop() {
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  rclcpp::Rate loop_rate(2000.0);
  while (rclcpp::ok() && run_.get()) {
    scalar_t horizon_time_ =
        min(2.0, max(0.5, gaitSchedulePtr_->current_gait_cycle()));

    auto mode_schedule_ptr = gaitSchedulePtr_->eval(horizon_time_);

    trajGenPtr_->update_current_state(estimatorPtr_->getQpos(),
                                      estimatorPtr_->getQvel());

    trajGenPtr_->update_mode_schedule(mode_schedule_ptr);

    atscImplPtr_->update_current_state(estimatorPtr_->getQpos(),
                                       estimatorPtr_->getQvel());

    atscImplPtr_->update_trajectory_reference(
        trajGenPtr_->get_trajectory_reference());

    atscImplPtr_->update_mode_schedule(mode_schedule_ptr);

    visPtr_->update_current_state(estimatorPtr_->getQpos(),
                                  estimatorPtr_->getQvel());
    visPtr_->update_trajectory_reference(
        trajGenPtr_->get_trajectory_reference());

    visPtr_->update_footholds(trajGenPtr_->get_footholds());

    loop_rate.sleep();
  }
}

} // namespace clear
