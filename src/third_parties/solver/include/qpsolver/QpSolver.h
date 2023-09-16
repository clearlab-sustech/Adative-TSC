#pragma once

#include <core/types.h>

namespace clear {
class QpSolver {

public:
  QpSolver(/* args */);
  ~QpSolver();

  matrix_t H, Aeq, C;
  vector_t g, beq, d;

private:
  /* data */
};

} // namespace clear
