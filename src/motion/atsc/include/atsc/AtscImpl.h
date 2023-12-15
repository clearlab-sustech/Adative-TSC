#pragma once
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/TrajectoriesArray.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trans/msg/actuator_cmds.hpp>

#include "asserts/trajectory/TrajectoriesArray.h"
#include "atsc/AdativeGain.h"
#include "atsc/WholeBodyController.h"

using namespace rclcpp;

namespace clear {
class AtscImpl {

public:
  AtscImpl(Node::SharedPtr nodeHandle, const std::string config_yaml);

  ~AtscImpl();

  void update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

  void update_trajectory_reference(
      std::shared_ptr<TrajectoriesArray> referenceTrajectoriesPtr);

  void update_mode_schedule(const std::shared_ptr<ModeSchedule> mode_schedule);

  trans::msg::ActuatorCmds::SharedPtr getCmds();

private:
  void publishCmds();

  void inner_loop();

  void adapative_gain_loop();

private:
  Node::SharedPtr nodeHandle_;
  rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
      actuators_cmds_pub_ptr_;

  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<AdaptiveGain> adaptiveGain_ptr_;
  std::shared_ptr<WholeBodyController> wbcPtr_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;
  Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;
  Buffer<std::shared_ptr<TrajectoriesArray>> refTrajPtrBuffer_;

  Buffer<std::shared_ptr<AdaptiveGain::FeedbackGain>> feedback_gain_buffer_;

  std::thread inner_loop_thread_, adapative_gain_thread_;
  Buffer<bool> run_;
  scalar_t freq_;

  std::string base_name, robot_name;
  std::vector<std::string> actuated_joints_name;
  Buffer<std::shared_ptr<ActuatorCommands>> actuator_commands_buffer;
};

} // namespace clear
