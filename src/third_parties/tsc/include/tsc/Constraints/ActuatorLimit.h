#ifndef TASKSPACECONTROL_ACTUATORLIMIT_H
#define TASKSPACECONTROL_ACTUATORLIMIT_H

#include "tsc/Constraints/LinearConstraints.h"

namespace clear {
class ActuatorLimit : public LinearConstraints {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ActuatorLimit(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &C();

  virtual const vector_t &c_lb();

  virtual const vector_t &c_ub();

  const vector_t &torque_lb();

  const vector_t &torque_ub();

private:
  matrix_t _C;
  vector_t _c_lb, _c_ub;
  vector_t _lb, _ub;
};
} // namespace clear

#endif // TASKSPACECONTROL_ACTUATORLIMIT_H
