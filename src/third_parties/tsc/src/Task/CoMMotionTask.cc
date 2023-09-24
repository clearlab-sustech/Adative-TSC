#include "tsc/Task/CoMMotionTask.h"

using namespace clear;

CoMMotionTask::CoMMotionTask(PinocchioInterface &robot, string name)
    : Task(robot, name) {
  _posRef.setZero();
  _velRef.setZero();
  _accRef.setZero();
  _Kp.setZero();
  _Kd.setZero();
  _Q.setIdentity();
}

void CoMMotionTask::update() {
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();

  acc_fb = _Kp * (_posRef - robot().getCoMPos()) +
           _Kd * (_velRef - robot().getCoMVel()) + _accRef;

  matrix_t A = robot().getJacobia_CoM() * S;
  vector_t a = acc_fb - robot().getCoMAcc();
  _H = A.transpose() * _Q * A;
  _g = -A.transpose() * _Q * a;
}

const matrix_t &CoMMotionTask::H() { return _H; }

const vector_t &CoMMotionTask::g() { return _g; }

matrix3_t &CoMMotionTask::Kp() { return _Kp; }

matrix3_t &CoMMotionTask::Kd() { return _Kd; }

vector3_t &CoMMotionTask::posRef() { return _posRef; }

vector3_t &CoMMotionTask::velRef() { return _velRef; }

vector3_t &CoMMotionTask::accRef() { return _accRef; }

matrix3_t &CoMMotionTask::weightMatrix() { return _Q; }