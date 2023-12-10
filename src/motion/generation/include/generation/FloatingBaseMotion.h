#pragma once

#include "generation/LinearInvertedPendulum.h"
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
class FloatingBaseMotion {

public:
  FloatingBaseMotion(
      Node::SharedPtr nodeHandle,
      std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
      std::shared_ptr<ReferenceBuffer> referenceTrajectoriesBuffer);

  ~FloatingBaseMotion();

  void generate();


private:
  void generate_reference();

  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> refTrajBuffer_;
  std::shared_ptr<LinearInvertedPendulum> lipm_ptr_;
  std::string base_name;
  std::vector<string> foot_names;
};

} // namespace clear
