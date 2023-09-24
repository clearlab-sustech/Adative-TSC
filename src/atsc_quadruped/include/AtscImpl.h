#pragma once
#include <core/misc/Buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <trans/msg/actuator_cmds.hpp>
#include <trans/msg/estimated_states.hpp>
#include <trans/msg/torch_mode.hpp>
#include <trans/msg/trajectory_array.hpp>
#include <tsc/tsc.h>

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

  void torch_mode_callback(const trans::msg::TorchMode::SharedPtr msg) const;

  void
  trajectories_callback(const trans::msg::TrajectoryArray::SharedPtr msg) const;

  void updateTask();

  void trajectoriesPreprocessing();

  void updatePinocchioInterface();

  void publishCmds();

  void inner_loop();

private:
  rclcpp::Subscription<trans::msg::EstimatedStates>::SharedPtr
      estimated_state_subscription_;
  rclcpp::Subscription<trans::msg::TorchMode>::SharedPtr
      torch_mode_subscription_;
  rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
      actuators_cmds_pub_ptr_;

  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  shared_ptr<TaskSpaceControl> tsc_ptr_;
  mutable Buffer<trans::msg::EstimatedStates::SharedPtr> estimated_state_buffer;
  mutable Buffer<trans::msg::TorchMode::SharedPtr> torch_mode_buffer;
  mutable Buffer<trans::msg::TrajectoryArray::SharedPtr> trajectories_buffer;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t dt_;

  std::shared_ptr<SE3MotionTask> floatingBaseTask;
  std::shared_ptr<RegularizationTask> regularizationTask;
  std::shared_ptr<NewtonEulerEq> ne_eq;
  std::shared_ptr<ContactPointsConstraints> maintainContact;
  std::shared_ptr<ContactForceConstraints> frictionCone;
  std::shared_ptr<ActuatorLimit> torqueLimit;
};

} // namespace clear
