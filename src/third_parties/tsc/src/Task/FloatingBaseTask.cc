

#include "tsc/Task/FloatingBaseTask.h"

using namespace clear;

FloatingBaseTask::FloatingBaseTask(PinocchioInterface &robot, string name)
    : Task(robot, name) {
  _spatialAccRef.setZero();
  _Q.setIdentity();
}

void FloatingBaseTask::update() {
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();
  robot().getJacobia_local(Task::name(), J);
  matrix_t A = J * S;

  if (_spatialAccRef.norm() > 200.0) {
    _spatialAccRef = 200.0 * _spatialAccRef.normalized();
  }

  a = _spatialAccRef - robot().getFrame6dAcc_local(Task::name()).toVector();

  _H = A.transpose() * _Q * A;
  _g = -A.transpose() * _Q * a;
}

const matrix_t &FloatingBaseTask::H() { return _H; }

const vector_t &FloatingBaseTask::g() { return _g; }

vector6_t &FloatingBaseTask::spatialAccRef() { return _spatialAccRef; }

scalar_t FloatingBaseTask::cost(vector_t &optimal_u) {
  return 0.5 * (optimal_u.transpose() * _H * optimal_u +
                _g.transpose() * optimal_u)[0];
}

vector6_t FloatingBaseTask::error(vector_t &optimal_u) {
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();
  robot().getJacobia_local(Task::name(), J);
  matrix_t A = J * S;
  return (A * optimal_u - a);
}

matrix6_t &FloatingBaseTask::weightMatrix() { return _Q; }
