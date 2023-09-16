#pragma once

/* QP-based Controller */
/* Variables x=[\qacc, \torque, \force]: (18 + 12 + 3 * 4) */

#include "less/FrictionCone.h"
#include "less/MaintainContactTask.h"
#include "less/NewtonEulerEquation.h"
#include "less/SE3Task.h"
#include "less/TorqueLimits.h"

#include <memory>
#include <pinocchio/PinocchioInterface.h>
#include <string>
#include <vector>

using namespace std;

namespace clear {
class AtscQuadruped {
public:
  AtscQuadruped(const std::string config_yaml);

  ~AtscQuadruped();

private:
  shared_ptr<PinocchioInterface> pinocchioInterface_ptr;
  shared_ptr<NewtonEulerEquation> newton_euler_eq_ptr;
  shared_ptr<MaintainContactTask> contact_ptr;
  shared_ptr<FrictionCone> friction_cone_ptr;
  shared_ptr<TorqueLimits> torque_limits_ptr;
  std::vector<std::shared_ptr<SE3Task>> se3_array_ptr;
};
} // namespace clear
