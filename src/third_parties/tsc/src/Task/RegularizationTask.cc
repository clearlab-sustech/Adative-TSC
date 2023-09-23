#include "Task/RegularizationTask.h"

// #define REGULARIZE_TORQUE

using namespace clear;

RegularizationTask::RegularizationTask(PinocchioInterface &robot, string name)
    : Task(robot, name) {
  w_acc_ = 1e-8;
  w_force_ = 1e-12;
}

void RegularizationTask::update() {
  _Q.setZero(n_var, n_var);
  _Q.diagonal().head(robot().nv()).fill(w_acc_);
  _Q.diagonal().segment(robot().nv(), robot().na()).fill(w_torque_);
  _Q.diagonal().tail(3 * robot().nc()).fill(w_force_);
  _g = vector_t::Zero(n_var);
}

const matrix_t &RegularizationTask::H() { return _H; }

const vector_t &RegularizationTask::g() { return _g; }

scalar_t &RegularizationTask::qaccWeight() { return w_acc_; }

scalar_t &RegularizationTask::forceWeight() { return w_force_; }

scalar_t &RegularizationTask::torqueWeight() { return w_torque_; }
