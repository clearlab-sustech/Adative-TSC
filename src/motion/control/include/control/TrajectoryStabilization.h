#pragma once
#include "control/WholeBodyController.h"
#include "control/ConstructVectorField.h"
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trans/msg/actuator_cmds.hpp>
#include <core/trajectory/ReferenceBuffer.h>

using namespace rclcpp;

namespace clear
{
  class TrajectoryStabilization
  {

  public:
    TrajectoryStabilization(
        Node::SharedPtr nodeHandle,
        std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
            robot_interface_ptr);

    ~TrajectoryStabilization();

    void updateCurrentState(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

    void updateMpcSolution(std::shared_ptr<ocs2::PrimalSolution> mpc_sol);

    void updateReferenceBuffer(std::shared_ptr<ReferenceBuffer> referenceBuffer);

    trans::msg::ActuatorCmds::SharedPtr getCmds();

  private:
    void publishCmds();

    void innerLoop();

    void adapative_gain_loop();

  private:
    Node::SharedPtr nodeHandle_;
    std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface>
        robot_interface_ptr_;

    rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
        actuators_cmds_pub_ptr_;

    Buffer<std::shared_ptr<ocs2::PrimalSolution>> mpc_sol_buffer;
    std::shared_ptr<WholeBodyController> wbcPtr_;
    std::shared_ptr<ConstructVectorField> vf_ptr_;
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;

    Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;
    std::shared_ptr<ReferenceBuffer> referenceBuffer_;

    std::thread inner_loop_thread_, vector_field_thread_;
    Buffer<bool> run_;
    scalar_t freq_;
    bool use_vector_field;
    Buffer<std::shared_ptr<ConstructVectorField::VectorFieldParam>>
        vf_param_buffer_;

    std::string base_name, robot_name;
    std::vector<std::string> actuated_joints_name;
    std::vector<scalar_t> joints_default_pos;
    Buffer<std::shared_ptr<ActuatorCommands>> actuator_commands_buffer;
  };

} // namespace clear
