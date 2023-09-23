#ifndef POPLAR_ANGULARMOMENTUMTASK_H
#define POPLAR_ANGULARMOMENTUMTASK_H

#include "Task/Task.h"

namespace clear {
class AngularMomentumTask : public Task {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AngularMomentumTask(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &H();

  virtual const vector_t &g();

  matrix3_t &Kp();

  vector3_t &ref();

  vector3_t &ref_dot();

  matrix3_t &weightMatrix();

private:
  matrix_t _H;
  matrix3_t _Kp, _Q;
  vector_t _g;
  vector3_t _ref, _ref_dot;
};
} // namespace clear

#endif // POPLAR_ANGULARMOMENTUMTASK_H
