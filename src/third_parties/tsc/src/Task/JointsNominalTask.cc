#include "Task/JointsNominalTask.h"

using namespace clear;

clear::JointsNominalTask::JointsNominalTask(PinocchioInterface &robot,
                                            string name)
    : Task(robot, name) {
  _q_n.resize(robot.na());
  _q_n.setZero();
  _Kp.resize(robot.na(), robot.na());
  _Kp.setZero();
  _Kd.resize(robot.na(), robot.na());
  _Kd.setZero();
  _Q.resize(robot.na(), robot.na());
  _Q.setZero();
}

void JointsNominalTask::update() {
  matrix_t S = matrix_t::Zero(robot().na(), n_var);
  S.middleCols(robot().nv() - robot().na(), robot().na()).setIdentity();
  vector_t qa_acc_des = _Kp * (_q_n - robot().qpos().tail(robot().na())) -
                        _Kd * robot().qvel().tail(robot().na());
  _H = S.transpose() * _Q * S;
  _g = -S.transpose() * _Q * qa_acc_des;
}

const matrix_t &JointsNominalTask::H() { return _H; }

const vector_t &JointsNominalTask::g() { return _g; }

matrix_t &JointsNominalTask::Kp() { return _Kp; }

matrix_t &JointsNominalTask::Kd() { return _Kd; }

matrix_t &JointsNominalTask::weightMatrix() { return _Q; }

vector_t &JointsNominalTask::norminalPosition() { return _q_n; }
