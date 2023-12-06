#pragma once
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/ReferenceBuffer.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trans/msg/actuator_cmds.hpp>

#include "asserts/trajectory/ReferenceBuffer.h"
#include "atsc/ConstructVectorField.h"
#include "atsc/WholeBodyController.h"

using namespace rclcpp;

namespace clear {
class TrajectoryStabilization {

public:
  TrajectoryStabilization(Node::SharedPtr nodeHandle);

  ~TrajectoryStabilization();

  void update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

  void update_trajectory_reference(
      std::shared_ptr<ReferenceBuffer> referenceTrajectoriesPtr);

private:
  void publishCmds();

  void inner_loop();

  void vector_field_loop();

private:
  Node::SharedPtr nodeHandle_;
  rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
      actuators_cmds_pub_ptr_;

  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ConstructVectorField> vf_construct_ptr_;
  std::shared_ptr<WholeBodyController> wbcPtr_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;
  Buffer<std::shared_ptr<ReferenceBuffer>> refTrajPtrBuffer_;

  Buffer<std::shared_ptr<ConstructVectorField::VectorFieldCoeffs>> vf_coeffs_buffer_;

  std::thread inner_loop_thread_, vector_field_thread_;
  Buffer<bool> run_;
  scalar_t freq_;

  std::string robot_name;
  std::vector<std::string> actuated_joints_name;
  std::shared_ptr<ActuatorCommands> actuator_commands_;
};

} // namespace clear
