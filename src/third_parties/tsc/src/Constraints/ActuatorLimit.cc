//
// Created by nimapng on 6/29/21.
//

#include "tsc/Constraints/ActuatorLimit.h"

using namespace clear;

ActuatorLimit::ActuatorLimit(PinocchioInterface &robot, string name)
    : LinearConstraints(robot, name, false) {
  vector_t effort_limit = robot.getModel().effortLimit.tail(robot.na());
  _lb = -effort_limit;
  _ub = effort_limit;

  std::cout << "torque limit: " << effort_limit.transpose() << "\n";

  _C.setZero(robot.na(), n_var);
  _C.middleCols(robot.nv(), robot.na()).setIdentity();

  _c_lb = _lb;
  _c_ub = _ub;
}

void ActuatorLimit::update() {}

const matrix_t &ActuatorLimit::C() { return _C; }

const vector_t &ActuatorLimit::c_lb() { return _c_lb; }

const vector_t &ActuatorLimit::c_ub() { return _c_ub; }

const vector_t &ActuatorLimit::torque_lb() { return _lb; }

const vector_t &ActuatorLimit::torque_ub() { return _ub; }
