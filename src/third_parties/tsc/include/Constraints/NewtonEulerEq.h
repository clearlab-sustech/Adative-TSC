#pragma once

#include "Constraints/LinearConstraints.h"

namespace clear {
class NewtonEulerEq : public LinearConstraints {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NewtonEulerEq(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &C();

  virtual const vector_t &c_lb();

  virtual const vector_t &c_ub();

private:
  matrix_t _C;
  vector_t _c_lb, _c_ub;
};

} // namespace clear
