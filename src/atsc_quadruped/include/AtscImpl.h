#pragma once
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trans/msg/actuator_cmds.hpp>
#include <trans/msg/estimated_states.hpp>
#include <trans/msg/mode_schedule_trans.hpp>
#include <trans/msg/trajectory_array.hpp>
#include <tsc/tsc.h>

#include "AdativeGain.h"
#include "TrajectoriesArray.h"

using namespace rclcpp;

namespace clear {
class AtscImpl : public Node {

public:
  AtscImpl(const std::string config_yaml);

  ~AtscImpl();

  void enable_adaptive_gain();

private:
  void estimated_state_callback(
      const trans::msg::EstimatedStates::SharedPtr msg) const;

  void mode_schedule_callback(
      const trans::msg::ModeScheduleTrans::SharedPtr msg) const;

  void
  trajectories_callback(const trans::msg::TrajectoryArray::SharedPtr msg) const;

  void updateFloatingBaseTask();

  void updateFootSwingBaseTask();

  void trajectoriesPreprocessing();

  void updatePinocchioInterface();

  void publishCmds();

  void inner_loop();

  void adapative_gain_loop();

  void inverse_kinematics();

  void prepare_cmds();

private:
  rclcpp::Subscription<trans::msg::EstimatedStates>::SharedPtr
      estimated_state_subscription_;
  rclcpp::Subscription<trans::msg::ModeScheduleTrans>::SharedPtr
      mode_schedule_subscription_;
  rclcpp::Subscription<trans::msg::TrajectoryArray>::SharedPtr
      trajectories_subscription_;
  rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
      actuators_cmds_pub_ptr_;

  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<TaskSpaceControl> tsc_ptr_;
  std::shared_ptr<AdaptiveGain> adaptiveGain_ptr_;

  mutable Buffer<trans::msg::EstimatedStates::SharedPtr> estimated_state_buffer;
  mutable Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;
  mutable Buffer<trans::msg::TrajectoryArray::SharedPtr>
      trajectories_msg_buffer_;
  mutable Buffer<bool> trajectories_updated_;
  Buffer<std::shared_ptr<TrajectoriesArray>> refTrajPtrBuffer_;
  Buffer<std::shared_ptr<AdaptiveGain::FeedbackGain>> feedback_gain_buffer_;

  std::thread inner_loop_thread_, adapative_gain_thread_;
  Buffer<bool> run_;
  scalar_t freq_;

  std::shared_ptr<FloatingBaseTask> floatingBaseTask;
  std::shared_ptr<RegularizationTask> regularizationTask;
  std::vector<std::shared_ptr<TranslationTask>> foot_task_array;
  std::shared_ptr<NewtonEulerEq> ne_eq;
  std::shared_ptr<ContactPointsConstraints> maintainContact;
  std::shared_ptr<ContactForceConstraints> frictionCone;
  std::shared_ptr<ActuatorLimit> torqueLimit;
  std::string base_name;
  std::shared_ptr<trans::msg::ActuatorCmds> actuator_commands_;
};

} // namespace clear
