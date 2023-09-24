#include "tsc/Constraints/NewtonEulerEq.h"

namespace clear {

NewtonEulerEq::NewtonEulerEq(PinocchioInterface &robot, string name)
    : LinearConstraints(robot, name, true) {}

void NewtonEulerEq::update() {
  _C.setZero(robot().nv(), n_var);
  _C.leftCols(robot().nv()) = robot().M();
  _C.middleCols(robot().nv(), robot().na())
      .bottomRows(robot().na())
      .diagonal()
      .fill(-1);
  auto mask = robot().getContactMask();
  for (size_t i = 0; i < robot().nc(); i++) {
    if (mask[i]) {
      matrix6x_t Ji;
      robot().getContactPointJacobia_localWorldAligned(i, Ji);
      _C.middleCols(robot().nv() + robot().na() + 3 * i, 3) =
          -Ji.topRows(3).transpose();
    }
  }
  _c_ub = -robot().nle();
  _c_lb = -robot().nle();
}

const matrix_t &NewtonEulerEq::C() { return _C; }

const vector_t &NewtonEulerEq::c_lb() { return _c_lb; }

const vector_t &NewtonEulerEq::c_ub() { return _c_ub; }

} // namespace clear
