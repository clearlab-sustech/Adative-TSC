#pragma once

#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/ReferenceBuffer.h>
#include <core/types.h>
#include <hpipm-cpp/hpipm-cpp.hpp>
#include <map>
#include <memory>
#include <pinocchio/Orientation.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <string>

#include "generation/LegLogic.h"

using namespace rclcpp;

namespace clear {
class FootholdOptimization {
public:
  FootholdOptimization(
      Node::SharedPtr nodeHandle,
      std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
      std::shared_ptr<ReferenceBuffer> referenceTrajectoriesBuffer);

  ~FootholdOptimization();

  void optimize();

private:
  void heuristic();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> refTrajBuffer_;
  std::map<std::string, std::pair<scalar_t, vector3_t>> footholds_;

  std::string base_name, robot_name;
  std::vector<std::string> foot_names;

  std::map<std::string, vector3_t>
      footholds_nominal_pos; // nominal footholds relative to nominal com pos
  scalar_t nominal_dz_;
  size_t nf;
};

} // namespace clear