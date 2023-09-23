//
// Created by nimapng on 6/11/21.
//

#include "Constraints/ContactForceConstraints.h"

using namespace clear;

ContactForceConstraints::ContactForceConstraints(PinocchioInterface &robot,
                                                 string name)
    : LinearConstraints(robot, name, false) {
  _mu = 0.5;
  _min = 0;
  _max = 500;
}

void ContactForceConstraints::update() {
  auto c_mask = robot().getContactMask();

  size_t nc_enable = std::count(c_mask.begin(), c_mask.end(), true);

  _C.resize(nc_enable * 5, n_var);
  _c_ub.resize(nc_enable * 5);
  _c_lb.resize(nc_enable * 5);

  matrix_t frictionPyramic(5, 3); // clang-format off
  frictionPyramic << 0, 0, 1,
                     1, 0, -_mu,
                    -1, 0, -_mu,
                     0, 1, -_mu,
                     0,-1, -_mu; // clang-format on
  _c_ub = Eigen::VectorXd::Zero(_C.rows());
  _c_lb = -1e16 * Eigen::VectorXd::Ones(_C.rows());

  int j = 0;
  for (int i = 0; i < robot().nc(); ++i) {
    if (c_mask[i]) {
      _C.block<5, 3>(5 * j, robot().nv() + 3 * i) = frictionPyramic;
      _c_lb(5 * j) = 0.0;
      _c_ub(5 * j) = 400.0;
      j++;
    }
  }
}

const matrix_t &ContactForceConstraints::C() { return _C; }

const vector_t &ContactForceConstraints::c_lb() { return _c_lb; }

const vector_t &ContactForceConstraints::c_ub() { return _c_ub; }

scalar_t &ContactForceConstraints::mu() { return _mu; }

scalar_t &ContactForceConstraints::min_force() { return _min; }

scalar_t &ContactForceConstraints::max_force() { return _max; }