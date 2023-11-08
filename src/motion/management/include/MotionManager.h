#pragma once

#include "DataVisualization.h"
#include "Initialization.h"
#include <atsc/AtscImpl.h>
#include <estimation/StateEstimationLKF.h>
#include <gait/GaitSchedule.h>
#include <generation/TrajectorGeneration.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unitree_hw/UnitreeHW.h>

using namespace rclcpp;

namespace clear {
class MotionManager : public Node {

public:
  MotionManager(std::string config_yaml);

  ~MotionManager();

  void init();

private:
  void inner_loop();

private:
  std::string config_yaml_;
  std::shared_ptr<StateEstimationLKF> estimatorPtr_;
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<TrajectorGeneration> trajGenPtr_;
  std::shared_ptr<AtscImpl> atscImplPtr_;
  std::shared_ptr<DataVisualization> visPtr_;
  std::shared_ptr<Initialization> intializationPtr_;
  std::shared_ptr<UnitreeHW> unitreeHWPtr_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
};

} // namespace clear
