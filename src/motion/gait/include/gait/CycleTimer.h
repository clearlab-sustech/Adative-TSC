#pragma once

#include <core/misc/Buffer.h>
#include <core/types.h>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

namespace clear {
class CycleTimer {
public:
  CycleTimer(Node::SharedPtr nodeHandle, scalar_t cycle_duration);

  ~CycleTimer();

  void timer_reset();

  scalar_t get_cycle_time();

private:
  void inner_loop();

private:
  Node::SharedPtr nodeHandle_;
  scalar_t cycle_duration_;
  Buffer<scalar_t> cycle_start_point_;
  Buffer<scalar_t> current_cycle_time_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
};

} // namespace clear
