#include "tsc/Task/AngularMomentumTask.h"

using namespace clear;

clear::AngularMomentumTask::AngularMomentumTask(PinocchioInterface &robot,
                                                string name)
    : Task(robot, name) {
  _Q.setZero();
  _Kp.setZero();
  _ref.setZero();
  _ref_dot.setZero();
}

void AngularMomentumTask::update() {
  matrix3x_t J_am = robot().getMomentumJacobia().bottomRows(3);
  vector3_t mdot_des = _Kp * (_ref - J_am * robot().qvel()) + _ref_dot -
                       robot().getMomentumTimeVariation().tail(3);
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();
  matrix_t Js = J_am * S;
  _H = Js.transpose() * _Q * Js;
  _g = -Js.transpose() * _Q * mdot_des;
}

const matrix_t &AngularMomentumTask::H() { return _H; }

const vector_t &AngularMomentumTask::g() { return _g; }

matrix3_t &AngularMomentumTask::Kp() { return _Kp; }

vector3_t &AngularMomentumTask::ref() { return _ref; }

vector3_t &AngularMomentumTask::ref_dot() { return _ref_dot; }

matrix3_t &AngularMomentumTask::weightMatrix() { return _Q; }