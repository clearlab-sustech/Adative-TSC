#pragma once

#include "tsc/Task/Task.h"

namespace clear {
class TranslationTask : public Task {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TranslationTask(PinocchioInterface &robot, string name);

  virtual void update();

  virtual const matrix_t &H();

  virtual const vector_t &g();

  matrix3_t &Kp();

  matrix3_t &Kd();

  vector3_t &posRef();

  vector3_t &velRef();

  vector3_t &accRef();

  scalar_t cost(vector_t &optimal_u);

  vector3_t error(vector_t &optimal_u);

  matrix3_t &weightMatrix();

private:
  matrix_t _H;
  matrix3_t _Kp, _Kd, _Q;
  vector_t _g;
  vector3_t _posRef, _velRef, _accRef, _a; 
};
} // namespace clear
