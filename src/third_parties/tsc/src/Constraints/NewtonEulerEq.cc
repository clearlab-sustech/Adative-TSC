#include "Constraints/NewtonEulerEq.h"

namespace clear {

NewtonEulerEq::NewtonEulerEq(PinocchioInterface &robot, string name)
    : LinearConstraints(robot, name, true) {
  _C.setZero(robot.nv(), n_var);
  _c_lb.setZero(robot.nv());
  _c_ub.setZero(robot.nv());
}

void NewtonEulerEq::update() {
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  _C.leftCols(robot().nv()) = robot().M();
  _C.middleCols(robot().nv(), robot().na()).bottomRows(robot().na()).diag
}

const matrix_t &NewtonEulerEq::C() { return _C; }

const vector_t &NewtonEulerEq::c_lb() { return _c_lb; }

const vector_t &NewtonEulerEq::c_ub() { return _c_ub; }

} // namespace clear
