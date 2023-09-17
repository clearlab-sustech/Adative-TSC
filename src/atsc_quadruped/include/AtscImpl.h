#pragma once
#include <core/misc/Buffer.h>
#include <less/AtscQuadruped.h>
#include <rclcpp/rclcpp.hpp>
#include <trans/msg/actuator_cmds.hpp>
#include <trans/msg/estimated_states.hpp>

#include <memory>
#include <string>

using namespace rclcpp;

namespace clear {
class AtscImpl : public Node {

public:
  AtscImpl(const std::string config_yaml);

  ~AtscImpl();

private:
  void estimated_state_callback(
      const trans::msg::EstimatedStates::SharedPtr msg) const;

  void inner_loop();

private:
  rclcpp::Subscription<trans::msg::EstimatedStates>::SharedPtr
      estimated_state_subscription_;
  rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
      actuators_cmds_pub_ptr_;

  shared_ptr<AtscQuadruped> atsc_ptr;
  mutable Buffer<trans::msg::EstimatedStates::SharedPtr> estimated_state_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t dt_;
};

} // namespace clear
