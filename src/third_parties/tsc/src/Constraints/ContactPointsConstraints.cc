//
// Created by nimapng on 6/11/21.
//

#include "Constraints/ContactPointsConstraints.h"

using namespace clear;

clear::ContactPointsConstraints::ContactPointsConstraints(
    PinocchioInterface &robot, string name)
    : LinearConstraints(robot, name, true) {}

void ContactPointsConstraints::update() {
  matrix6x_t Jc;
  robot().getJacobia_localWorldAligned(LinearConstraints::name(), Jc);
  auto acc = robot().getFrame6dAcc_localWorldAligned(LinearConstraints::name());

  matrix_t S;
  S.setZero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();
  _C = Jc.topRows(3) * S;
  _c_ub = -acc.linear();
  _c_lb = -acc.linear();
}

const matrix_t &ContactPointsConstraints::C() { return _C; }

const vector_t &ContactPointsConstraints::c_lb() { return _c_lb; }

const vector_t &ContactPointsConstraints::c_ub() { return _c_ub; }
