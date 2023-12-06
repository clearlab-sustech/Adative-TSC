#pragma once

#include "DataVisualization.h"
#include "Initialization.h"
#include <atsc/TrajectoryStabilization.h>
#include <estimation/StateEstimationLKF.h>
#include <gait/GaitSchedule.h>
#include <generation/TrajectorGeneration.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;

namespace clear {
class MotionManager : public Node {

public:
  MotionManager();

  ~MotionManager();

  void init();

private:
  void inner_loop();

private:
  std::shared_ptr<StateEstimationLKF> estimatorPtr_;
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<TrajectorGeneration> trajGenPtr_;
  std::shared_ptr<TrajectoryStabilization> tsImplPtr_;
  std::shared_ptr<DataVisualization> visPtr_;
  std::shared_ptr<Initialization> intializationPtr_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
};

} // namespace clear
