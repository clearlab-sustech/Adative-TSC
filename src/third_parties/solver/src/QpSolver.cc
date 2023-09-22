#include "qpsolver/QpSolver.h"

namespace clear {

void QpSolver::QpSolverSettings::checkSettings() const {
  if (iter_max < 0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.iter_max must be non-negative");
  }
  if (alpha_min <= 0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.alpha_min must be positive");
  }
  if (alpha_min > 1.0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.alpha_min must be less than 1.0");
  }
  if (mu0 <= 0.0) {
    throw std::runtime_error("OcpQpIpmSolverSettings.mu0 must be positive");
  }
  if (tol_stat <= 0.0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.tol_stat must be positive");
  }
  if (tol_eq <= 0.0) {
    throw std::runtime_error("OcpQpIpmSolverSettings.tol_eq must be positive");
  }
  if (tol_ineq <= 0.0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.tol_ineq must be positive");
  }
  if (tol_comp <= 0.0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.tol_comp must be positive");
  }
  if (reg_prim < 0.0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.reg_prim must be non-negative");
  }
  if (reg_prim < 0.0) {
    throw std::runtime_error(
        "OcpQpIpmSolverSettings.reg_prim must be non-negative");
  }
}

QpSolver::QpSolver(const DimsSpec &dims) : dims_(dims) {

  dim_size = d_dense_qp_dim_memsize();
  dim_mem = malloc(dim_size);
  d_dense_qp_dim_create(&dims_, dim_mem);

  hpipm_size_t qp_size = d_dense_qp_memsize(&dims_);
  void *qp_mem = malloc(qp_size);
  d_dense_qp_create(&dims_, &qp, qp_mem);

  qp_sol_size = d_dense_qp_sol_memsize(&dims_);
  qp_sol_mem = malloc(qp_sol_size);

  ipm_arg_size = d_dense_qp_ipm_arg_memsize(&dims_);
  void *ipm_arg_mem = malloc(ipm_arg_size);

  d_dense_qp_ipm_arg_create(&dims_, &arg, ipm_arg_mem);

  d_dense_qp_ipm_arg_set_default(hpipm_mode::SPEED, &arg);

  d_dense_qp_ipm_arg_set_mu0(&mu0, &arg);
  d_dense_qp_ipm_arg_set_iter_max(&iter_max, &arg);
  d_dense_qp_ipm_arg_set_alpha_min(&alpha_min, &arg);
  d_dense_qp_ipm_arg_set_mu0(&mu0, &arg);
  d_dense_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
  d_dense_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
  d_dense_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
  d_dense_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
  d_dense_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
  d_dense_qp_ipm_arg_set_reg_dual(&reg_dual, &arg);
  d_dense_qp_ipm_arg_set_warm_start(&warm_start, &arg);
  d_dense_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
  d_dense_qp_ipm_arg_set_split_step(&split_step, &arg);
}

QpSolver::~QpSolver() {
  free(dim_mem);
  free(qp_mem);
  free(qp_sol_mem);
  free(ipm_arg_mem);
  free(ipm_mem);

  free(v);
}

void QpSolver::update(Eigen::Ref<matrix_t> H, Eigen::Ref<vector_t> g,
                      Eigen::Ref<matrix_t> A, Eigen::Ref<vector_t> b,
                      Eigen::Ref<matrix_t> C, Eigen::Ref<vector_t> lb,
                      Eigen::Ref<vector_t> ub) {
  this->H = H;
  this->g = g;
  this->A = A;
  this->b = b;
  this->C = C;
  this->lb = lb;
  this->ub = ub;
}

vector_t QpSolver::solve() {
  d_dense_qp_set_all(H.data(), g.data(), A.data(), b.data(), NULL, NULL, NULL,
                     C.data(), lb.data(), ub.data(), NULL, NULL, NULL, NULL,
                     NULL, NULL, NULL, &qp);

  d_dense_qp_sol_create(&dims_, &qp_sol, qp_sol_mem);

  sol.setZero(dims_.nv);

  return sol;
}

} // namespace clear
