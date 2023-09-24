#ifndef TASKSPACECONTROL_CONTACTPOINTSCONSTRAINTS_H
#define TASKSPACECONTROL_CONTACTPOINTSCONSTRAINTS_H

#include "tsc/Constraints/LinearConstraints.h"

namespace clear {
class ContactPointsConstraints : public LinearConstraints {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ContactPointsConstraints(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &C();

  virtual const vector_t &c_lb();

  virtual const vector_t &c_ub();

private:
  matrix_t _C;
  vector_t _c_lb, _c_ub;
};
} // namespace clear

#endif // TASKSPACECONTROL_CONTACTPOINTSCONSTRAINTS_H
