#pragma once

#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/TrajectoriesArray.h>
#include <core/types.h>
#include <pinocchio/Orientation.h>
#include <pinocchio/PinocchioInterface.h>

#include <hpipm-cpp/hpipm-cpp.hpp>
#include <map>
#include <memory>
#include <rcpputils/asserts.hpp>

#include "generation/LegLogic.h"

namespace clear {
class FootholdOptimization {
public:
  FootholdOptimization(
      std::string config_yaml,
      std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
      std::shared_ptr<TrajectoriesArray> referenceTrajectoriesBuffer);

  ~FootholdOptimization();

  std::map<std::string, std::pair<scalar_t, vector3_t>>
  optimize(scalar_t t, const std::shared_ptr<ModeSchedule> mode_schedule);

private:
  void heuristic(scalar_t t, const std::shared_ptr<ModeSchedule> mode_schedule);

private:
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::string base_name, robot_name;
  std::vector<std::string> foot_names;

  std::shared_ptr<TrajectoriesArray> referenceTrajectoriesBuffer_;
  std::map<std::string, std::pair<scalar_t, vector3_t>> footholds_; // footholds
  std::map<std::string, vector3_t>
      footholds_nominal_pos; // nominal footholds relative to nominal com pos
  scalar_t nominal_dz_;
  size_t nf;
};

} // namespace clear