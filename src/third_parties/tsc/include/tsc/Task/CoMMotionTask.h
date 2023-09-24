//
// Created by nimapng on 6/11/21.
//

#ifndef TASKSPACECONTROL_COMMOTIONTASK_H
#define TASKSPACECONTROL_COMMOTIONTASK_H

#include "tsc/Task/Task.h"

namespace clear {
class CoMMotionTask : public Task {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CoMMotionTask(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &H();

  virtual const vector_t &g();

  matrix3_t &Kp();

  matrix3_t &Kd();

  vector3_t &posRef();

  vector3_t &velRef();

  vector3_t &accRef();

  matrix3_t &weightMatrix();

private:
  matrix_t _H;
  matrix3_t _Kp, _Kd, _Q;
  vector_t _g;
  vector3_t _posRef, _velRef, _accRef, acc_fb; // acc_fb , feedback term
};
} // namespace clear

#endif // TASKSPACECONTROL_COMMOTIONTASK_H
