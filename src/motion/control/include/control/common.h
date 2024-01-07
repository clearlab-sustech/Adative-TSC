#pragma once

#include <core/types.h>

namespace clear {
struct VectorFieldParam {
  matrix_t K;
  vector_t b;
  vector_t force_des;
};

struct ActuatorCommands {
  vector_t Kp;
  vector_t Kd;
  vector_t pos;
  vector_t vel;
  vector_t torque;

  void setZero(size_t n) {
    Kp.setZero(n);
    Kd.setZero(n);
    pos.setZero(n);
    vel.setZero(n);
    torque.setZero(n);
  }
};

} // namespace clear
