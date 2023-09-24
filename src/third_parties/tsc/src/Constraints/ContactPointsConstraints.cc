//
// Created by nimapng on 6/11/21.
//

#include "tsc/Constraints/ContactPointsConstraints.h"

using namespace clear;

clear::ContactPointsConstraints::ContactPointsConstraints(
    PinocchioInterface &robot, string name)
    : LinearConstraints(robot, name, true) {}

void ContactPointsConstraints::update() {
  auto c_mask = robot().getContactMask();
  size_t nc_enable = std::count(c_mask.begin(), c_mask.end(), true);

  matrix_t S;
  S.setZero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();

  _C = matrix_t::Zero(3 * nc_enable, n_var);
  _c_ub.setZero(3 * nc_enable);

  size_t j = 0;
  const auto contact_points = robot().getContactPoints();
  for (size_t i = 0; i < robot().nc(); i++) {
    if (c_mask[i]) {
      matrix6x_t Ji;
      robot().getContactPointJacobia_localWorldAligned(i, Ji);
      _C.middleRows(3 * j, 3) = Ji.topRows(3) * S;
      auto acc = robot().getFrame6dAcc_localWorldAligned(contact_points[i]);
      _c_ub.segment(3 * j, 3) = -acc.linear();
      j++;
    }
  }
  _c_lb = _c_ub;
}

const matrix_t &ContactPointsConstraints::C() { return _C; }

const vector_t &ContactPointsConstraints::c_lb() { return _c_lb; }

const vector_t &ContactPointsConstraints::c_ub() { return _c_ub; }
