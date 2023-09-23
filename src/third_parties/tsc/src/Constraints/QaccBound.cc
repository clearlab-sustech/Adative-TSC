//
// Created by nimapng on 6/29/21.
//

#include "Constraints/QaccBound.h"

using namespace clear;

clear::QaccBound::QaccBound(PinocchioInterface &robot, string name)
    : LinearConstraints(robot, name) {
  _lb.resize(robot.nv());
  _lb.fill(-1e3);
  _ub.resize(robot.nv());
  _ub.fill(1e3);
}

void QaccBound::update() {
  _C.resize(robot().nv(), n_var);
  _C.leftCols(robot().nv()).setIdentity();
  assert((_ub - _lb).all() > 0);
  _c_lb = _lb;
  _c_ub = _ub;
}

const matrix_t &QaccBound::C() { return _C; }

const vector_t &QaccBound::c_lb() { return _c_lb; }

const vector_t &QaccBound::c_ub() { return _c_ub; }

vector_t &QaccBound::lb() { return _lb; }

vector_t &QaccBound::ub() { return _ub; }
