#pragma once

#include "gait/CycleTimer.h"
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <core/misc/Lookup.h>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <trans/msg/mode_schedule_trans.hpp>
#include <trans/srv/gait_switch.hpp>
#include <yaml-cpp/yaml.h>

using namespace rclcpp;

namespace clear {

class GaitSchedule {
public:
  GaitSchedule(Node::SharedPtr nodeHandle);

  ~GaitSchedule();

  void switch_gait(std::string gait_name);

  std::shared_ptr<ModeSchedule> eval(scalar_t time_period);

  size_t current_mode();

  scalar_t current_gait_cycle();

  std::string get_current_gait_name();

private:
  std::shared_ptr<ModeSchedule> loadGait(const YAML::Node node,
                                         const std::string &gait_name);

  void
  gait_switch(const std::shared_ptr<trans::srv::GaitSwitch::Request> request,
              std::shared_ptr<trans::srv::GaitSwitch::Response> response);

  void check_gait_transition();

private:
  Node::SharedPtr nodeHandle_;
  rclcpp::Service<trans::srv::GaitSwitch>::SharedPtr gait_service_;
  std::string robot_name;

  std::vector<std::string> gait_list;
  Buffer<std::string> current_gait_;
  Buffer<std::string> gait_buffer_;
  Buffer<scalar_t> transition_time_;
  std::map<std::string, std::shared_ptr<ModeSchedule>> gait_map_;

  std::shared_ptr<CycleTimer> cycle_timer_;

  std::thread check_transition_thread_;
  Buffer<bool> check_;
  Buffer<bool> in_transition_;
};

} // namespace clear
