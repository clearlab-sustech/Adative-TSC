#ifndef TASKSPACECONTROL_QACCBOUND_H
#define TASKSPACECONTROL_QACCBOUND_H

#include "tsc/Constraints/LinearConstraints.h"

namespace clear {
class QaccBound : public LinearConstraints {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit QaccBound(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &C();

  virtual const vector_t &c_lb();

  virtual const vector_t &c_ub();

  vector_t& lb();

  vector_t& ub();

private:
  matrix_t _C;
  vector_t _c_lb, _c_ub;
  vector_t _lb, _ub;
};
} // namespace clear

#endif // TASKSPACECONTROL_QACCBOUND_H
