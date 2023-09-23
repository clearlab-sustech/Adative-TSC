#ifndef POPLAR_JOINTSNOMINALTASK_H
#define POPLAR_JOINTSNOMINALTASK_H

#include "Task/Task.h"

namespace clear {
class JointsNominalTask : public Task {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  JointsNominalTask(PinocchioInterface &robot, string name);

  void update() override;

  virtual const matrix_t &H();

  virtual const vector_t &g();

  matrix_t &Kp();

  matrix_t &Kd();

  matrix_t &weightMatrix();

  vector_t &norminalPosition();

private:
  matrix_t _H, _Q, _Kp, _Kd;
  vector_t _g, _q_n;
};
} // namespace clear

#endif // POPLAR_JOINTNORMINALTASK_H
