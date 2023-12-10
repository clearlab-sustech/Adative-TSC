#include <MotionManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

MotionManager::MotionManager() : Node("MotionManager") {
  this->declare_parameter("/config_file", "");
}

MotionManager::~MotionManager() {
  run_.push(false);
  inner_loop_thread_.join();
}

void MotionManager::init() {

  intializationPtr_ =
      std::make_shared<Initialization>(this->shared_from_this());

  intializationPtr_->reset_simulation();
  rclcpp::spin_some(this->shared_from_this());

  estimatorPtr_ =
      std::make_shared<StateEstimationLKF>(this->shared_from_this());

  gaitSchedulePtr_ = std::make_shared<GaitSchedule>(this->shared_from_this());

  trajGenPtr_ = std::make_shared<TrajectorGeneration>(this->shared_from_this());

  // tsImplPtr_ = std::make_shared<TrajectoryStabilization>(this->shared_from_this());

  visPtr_ = std::make_shared<DataVisualization>(this->shared_from_this());
  visPtr_->set_trajectory_reference(trajGenPtr_->get_trajectory_reference());

  inner_loop_thread_ = std::thread(&MotionManager::inner_loop, this);
  run_.push(true);
}

void MotionManager::inner_loop() {
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  rclcpp::Rate loop_rate(500.0);
  const scalar_t ts = this->now().seconds();
  while (rclcpp::ok() && run_.get()) {
    if (this->now().seconds() > ts + 0.05) {
      gaitSchedulePtr_->switch_gait("walk");
    }

    if (gaitSchedulePtr_->get_current_gait_name() == "walk") {
      trajGenPtr_->setVelCmd(vector3_t(0.2, 0.0, 0.0), 0.0);
    } else {
      trajGenPtr_->setVelCmd(vector3_t(0.0, 0.0, 0.0), 0.0);
    }

    scalar_t horizon_time_ =
        min(2.0, max(0.5, gaitSchedulePtr_->current_gait_cycle()));

    auto mode_schedule_ptr = gaitSchedulePtr_->eval(horizon_time_);

    trajGenPtr_->update_current_state(estimatorPtr_->getQpos(),
                                      estimatorPtr_->getQvel());

    trajGenPtr_->update_mode_schedule(mode_schedule_ptr);

    /* tsImplPtr_->update_current_state(estimatorPtr_->getQpos(),
                                       estimatorPtr_->getQvel());

    tsImplPtr_->update_trajectory_reference(
        trajGenPtr_->get_trajectory_reference()); */

    visPtr_->update_current_state(estimatorPtr_->getQpos(),
                                  estimatorPtr_->getQvel());

    loop_rate.sleep();
  }
}

} // namespace clear
