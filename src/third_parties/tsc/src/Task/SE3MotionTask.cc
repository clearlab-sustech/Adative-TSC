

#include "Task/SE3MotionTask.h"

using namespace clear;

SE3MotionTask::SE3MotionTask(PinocchioInterface &robot, string name)
    : Task(robot, name) {
  _SE3Ref.setIdentity();
  _spatialVelRef.setZero();
  _spatialAccRef.setZero();
  _Kp.setZero();
  _Kd.setZero();
  _Q.setIdentity();
}

void SE3MotionTask::update() {
  matrix_t S = matrix_t::Zero(robot().nv(), n_var);
  S.leftCols(robot().nv()).setIdentity();
  robot().getJacobia_local(Task::name(), J);
  _SE3 = robot().getFramePose(Task::name());
  acc_fb = _Kp * log6(_SE3.actInv(_SE3Ref)).toVector() +
           _Kd * (_spatialVelRef -
                  robot().getFrame6dVel_local(Task::name()).toVector()) +
           _spatialAccRef;
  A = J * S;
  a = acc_fb - robot().getFrame6dAcc_local(Task::name()).toVector();
  _H = A.transpose() * _Q * A;
  _g = -A.transpose() * _Q * a;
}

const matrix_t &SE3MotionTask::H() { return _H; }

const vector_t &SE3MotionTask::g() { return _g; }

matrix6_t &SE3MotionTask::Kp() { return _Kp; }

matrix6_t &SE3MotionTask::Kd() { return _Kd; }

pin::SE3 &SE3MotionTask::SE3Ref() { return ref(_SE3Ref); }

vector6_t &SE3MotionTask::spatialVelRef() { return _spatialVelRef; }

vector6_t &SE3MotionTask::spatialAccRef() { return _spatialAccRef; }

scalar_t SE3MotionTask::cost(vector_t &optimal_u) {
  return 0.5 * (optimal_u.transpose() * _H * optimal_u +
                _g.transpose() * optimal_u)[0];
}

vector6_t SE3MotionTask::error(vector_t &optimal_u) {
  return (A * optimal_u - a);
}

matrix6_t &SE3MotionTask::weightMatrix() { return _Q; }
