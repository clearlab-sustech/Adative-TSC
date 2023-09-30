#pragma once

#include "trans/srv/simulation_reset.hpp"
#include <core/types.h>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

namespace clear {
class Initialization {

public:
  Initialization(Node::SharedPtr nodeHandle, std::string config_yaml);

  ~Initialization();

  void reset_simulation();

private:
  Node::SharedPtr nodeHandle_;
  std::string config_yaml_;

  Client<trans::srv::SimulationReset>::SharedPtr reset_state_client_;
};
} // namespace clear