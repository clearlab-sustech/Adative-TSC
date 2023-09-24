

#include "tsc/Constraints/LinearConstraints.h"

using namespace clear;

LinearConstraints::LinearConstraints(PinocchioInterface &robot, string name,
                                     bool isEqual)
    : _robot(robot), _isEqual(isEqual), _enable(true), n_var(0), _name(name) {
  n_var = robot.nv() + robot.na() + 3 * robot.nc();
}

const string &LinearConstraints::name() { return _name; }

PinocchioInterface &LinearConstraints::robot() { return _robot; }

bool LinearConstraints::isEqual() { return _isEqual; }

void LinearConstraints::errPrint(vector_t &u) {
  if (isEqual()) {
    cout << name() << " equality err: " << (C() * u - c_ub()).norm() << endl;
  } else {
    cout << name() << " inequality ub err: " << (c_ub() - C() * u).transpose()
         << endl;
    cout << name() << " inequality lb err: " << (c_lb() - C() * u).transpose()
         << endl;
  }
}

bool LinearConstraints::is_enable() { return _enable; }

void LinearConstraints::enable() { _enable = true; }

void LinearConstraints::disable() { _enable = false; }
