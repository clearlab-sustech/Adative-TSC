#pragma once

#include "gait/CycleTimer.h"
#include "gait/ModeSchedule.h"
#include "gait/MotionPhaseDefinition.h"
#include "trans/msg/torch_mode.hpp"

#include <core/misc/Buffer.h>
#include <core/misc/Lookup.h>
#include <map>
#include <mutex>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

using namespace rclcpp;

namespace clear {

class GaitSchedule : public Node {
public:
  GaitSchedule(std::string config_yaml);

  ~GaitSchedule();

private:
  std::shared_ptr<ModeSchedule> eval(scalar_t time_period);

  void switch_gait(std::string gait_name);

  std::shared_ptr<ModeSchedule> loadGait(const YAML::Node node,
                                         const std::string &gait_name);

  void check_gait_transition();

  size_t current_mode();

  scalar_t current_gait_cycle();

  std::string get_current_gait_name();

  void inner_loop();

private:
  std::vector<std::string> gait_list;
  Buffer<std::string> current_gait_;
  Buffer<std::string> gait_buffer_;
  Buffer<scalar_t> transition_time_;
  std::map<std::string, std::shared_ptr<ModeSchedule>> gait_map_;

  std::shared_ptr<CycleTimer> cycle_timer_;

  std::thread check_transition_thread_;
  Buffer<bool> check_;
  Buffer<bool> in_transition_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;

  rclcpp::Publisher<trans::msg::TorchMode>::SharedPtr torch_mode_publisher_;
};

} // namespace clear
