#pragma once

#include <core/types.h>

#include <blasfeo_d_aux_ext_dep.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_d_dense_qp_utils.h>
#include <hpipm_timing.h>

namespace clear {

typedef d_dense_qp_sol QpSolution;

typedef d_dense_qp_ipm_arg SolverConfig;

class QpSolver {

public:
  struct QpSolverSettings {
  public:
    ///
    /// @brief Solver mode. Default is hpipm_mode::SPEED.
    ///
    hpipm_mode mode = hpipm_mode::SPEED;

    ///
    /// @brief Maximum number of iterations. Must be nen-negative. Default
    /// is 15.
    ///
    int iter_max = 30;

    ///
    /// @brief Minimum step size. Must be positive and less than 1.0. Default
    /// is 1.0e-08.
    ///
    double alpha_min = 1.0e-08;

    ///
    /// @brief Initial barrier parameter. Must be positive. Default is 1.0e+02.
    ///
    double mu0 = 1.0e+02;

    ///
    /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
    ///
    double tol_stat = 1.0e-04;

    ///
    /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
    ///
    double tol_eq = 1.0e-04;

    ///
    /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
    ///
    double tol_ineq = 1.0e-04;

    ///
    /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
    ///
    double tol_comp = 1.0e-04; // convergence criteria

    ///
    /// @brief Regularization term. Must be non-negative. Default is 1.0e-12.
    ///
    double reg_prim = 1.0e-12;

    ///
    /// @brief Regularization term. reg of dual hessian
    ///
    double reg_dual = 1.0e-8;

    ///
    /// @brief Warm start flag (0: disable, 1: enable). Default is 0.
    ///
    int warm_start = 0;

    ///
    /// @brief Prediction-correction flag (0: disable, 1: enable). Default is 1.
    ///
    int pred_corr = 1;

    ///
    /// @brief Square-root Riccati flag (0: disable, 1: enable). Default is 1.
    ///
    int ric_alg = 0;

    ///
    /// @brief Use different step for primal and dual variables (0: disable, 1:
    /// enable). Default is 0.
    ///
    int split_step = 1;

    bool verbose = false; // print debug info

    ///
    /// @brief Check the settings. If something is wrong, throws an exception.
    ///
    void checkSettings() const;
  };

  class DimsSpec : public d_dense_qp_dim {

  public:
    DimsSpec() {
      nv = 0;  // number of variables
      ne = 0;  // number of equality constraints
      nb = 0;  // number of box constraints
      ng = 0;  // number of general constraints
      nsb = 0; // number of softened box constraints
      nsg = 0; // number of softened general constraints
      ns = 0;  // number of softened constraints (nsb+nsg)
      memsize = 0;
    }
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QpSolver(const DimsSpec &dims, QpSolverSettings settings);

  ~QpSolver();

  void update(Eigen::Ref<matrix_t> H, Eigen::Ref<vector_t> g,
              Eigen::Ref<matrix_t> A, Eigen::Ref<vector_t> b,
              Eigen::Ref<matrix_t> C, Eigen::Ref<vector_t> lb,
              Eigen::Ref<vector_t> ub);

  vector_t solve();

private:
  DimsSpec dims_;
  matrix_t H, A, C;
  vector_t g, b, lb, ub, sol;

  hpipm_size_t dim_size;
  struct d_dense_qp qp;
  void *dim_mem;
  void *qp_mem;

  hpipm_size_t qp_sol_size;
  void *qp_sol_mem;
  QpSolution qp_sol;

  hpipm_size_t ipm_arg_size;
  SolverConfig arg;
  void *ipm_arg_mem;

  hpipm_size_t ipm_size;
  void *ipm_mem;
  struct d_dense_qp_ipm_ws workspace;

  bool verbose;

  void print(int hpipm_status);
};

} // namespace clear
