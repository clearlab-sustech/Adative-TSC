#include "qpsolver/QpSolver.h"
#include <iostream>

using namespace clear;

int main() {
  clear::DimsSpec dims;
  dims.nv = 2;
  dims.ne = 1;
  dims.ng = 1;
  dims.nb = 1;
  dims.ns = 0;
  dims.nsb = 0;
  dims.nsg = 0;

  clear::QpSolver::QpSolverSettings settings;
  settings.verbose = true;

  clear::QpSolver solver(dims, settings);
  matrix_t H(2, 2);
  H.setZero();
  H << 5.0, 3.0, 2.0, 3.0;

  vector_t g(2);
  g << -3.0, 2.0;
  matrix_t A(1, 2);
  A << 2.0, -1.0;
  vector_t b(1);
  b << 2.0;
  matrix_t C(1, 2);
  C << 1.0, 2.0;
  vector_t lb(1);
  lb << -12.0;
  vector_t ub(1);
  ub << 11.0;

  solver.update(H, g, A, b, C, lb, ub);
  auto sol = solver.solve();
  std::cout << "solution: " << sol.transpose() << std::endl;
  return 0;
}