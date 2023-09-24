#ifndef TASKSPACECONTROL_REGULARIZATIONTASK_H
#define TASKSPACECONTROL_REGULARIZATIONTASK_H

#include "tsc/Task/Task.h"

namespace clear {
class RegularizationTask : public Task {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RegularizationTask(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &H();

  virtual const vector_t &g();

  scalar_t &forceWeight();

  scalar_t &qaccWeight();

  scalar_t &torqueWeight();

private:
  matrix_t _H;
  vector_t _g;
  scalar_t w_acc_, w_force_, w_torque_;
};
} // namespace clear

#endif // TASKSPACECONTROL_REGULARIZATIONTASK_H
