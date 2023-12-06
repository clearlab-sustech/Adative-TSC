#pragma once

#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/ReferenceBuffer.h>
#include <core/misc/Buffer.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;

namespace clear {
class SwingTrajectory {

public:
  SwingTrajectory(Node::SharedPtr nodeHandle,
                  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
                  std::shared_ptr<ReferenceBuffer> referenceTrajectoriesBuffer);

  ~SwingTrajectory();

  void generate();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> refTrajBuffer_;
  std::vector<string> foot_names;
  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_start_;
  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_end_;
  legged_robot::contact_flag_t contact_flag_last;
};

} // namespace clear
