

#include "tsc/Task/TranslationTask.h"

using namespace clear;

TranslationTask::TranslationTask(PinocchioInterface &robot, string name)
    : Task(robot, name) {
  _posRef.setIdentity();
  _velRef.setZero();
  _accRef.setZero();
  _Kp.setZero();
  _Kd.setZero();
  _Q.setIdentity();
}

void TranslationTask::update() {
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();
  matrix6x_t J;
  robot().getJacobia_localWorldAligned(Task::name(), J);
  auto pose = robot().getFramePose(Task::name());
  vector3_t acc_fb =
      _Kp * (_posRef - pose.translation()) +
      _Kd * (_velRef -
             robot().getFrame6dVel_localWorldAligned(Task::name()).linear()) +
      _accRef;
  if (acc_fb.norm() > 200.0) {
    acc_fb = 200.0 * acc_fb.normalized();
  }
  matrix_t A = J.topRows(3) * S;
  _a = acc_fb - robot().getFrame6dVel_localWorldAligned(Task::name()).linear();

  _H = A.transpose() * _Q * A;
  _g = -A.transpose() * _Q * acc_fb;
}

const matrix_t &TranslationTask::H() { return _H; }

const vector_t &TranslationTask::g() { return _g; }

matrix3_t &TranslationTask::Kp() { return _Kp; }

matrix3_t &TranslationTask::Kd() { return _Kd; }

vector3_t &TranslationTask::posRef() { return _posRef; }

vector3_t &TranslationTask::velRef() { return _velRef; }

vector3_t &TranslationTask::accRef() { return _accRef; }

scalar_t TranslationTask::cost(vector_t &optimal_u) {
  return 0.5 * (optimal_u.transpose() * _H * optimal_u +
                _g.transpose() * optimal_u)[0];
}

vector3_t TranslationTask::error(vector_t &optimal_u) {
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();
  matrix6x_t J;
  robot().getJacobia_localWorldAligned(Task::name(), J);
  return (J.topRows(3) * S * optimal_u - _a);
}

matrix3_t &TranslationTask::weightMatrix() { return _Q; }
