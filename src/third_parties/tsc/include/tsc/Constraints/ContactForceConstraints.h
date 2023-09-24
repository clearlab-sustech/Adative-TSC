#ifndef TASKSPACECONTROL_CONTACTFORCECONSTRAINTS_H
#define TASKSPACECONTROL_CONTACTFORCECONSTRAINTS_H

#include "tsc/Constraints/LinearConstraints.h"

namespace clear {
class ContactForceConstraints : public LinearConstraints {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ContactForceConstraints(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &C();

  virtual const vector_t &c_lb();

  virtual const vector_t &c_ub();

  scalar_t &mu();

  scalar_t &min_force();

  scalar_t &max_force();

private:
  matrix_t _C;
  vector_t _c_lb, _c_ub;
  scalar_t _mu, _min, _max;
};
} // namespace clear

#endif // TASKSPACECONTROL_CONTACTFORCECONSTRAINTS_H
