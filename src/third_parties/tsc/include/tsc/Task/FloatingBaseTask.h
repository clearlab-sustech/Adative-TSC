
#pragma once

#include "tsc/Task/Task.h"

namespace clear {
class FloatingBaseTask : public Task {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FloatingBaseTask(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &H();

  virtual const vector_t &g();

  vector6_t &spatialAccRef();

  scalar_t cost(vector_t &optimal_u);

  vector6_t error(vector_t &optimal_u);

  matrix6_t &weightMatrix();

private:
  matrix_t _H;
  matrix6_t _Q;
  matrix6x_t J;
  vector_t _g, a;
  vector6_t _spatialAccRef;
};
} // namespace clear

