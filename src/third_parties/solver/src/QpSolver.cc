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

QpSolver::QpSolver(const DimsSpec &dims, QpSolverSettings settings)
    : dims_(dims) {

  dim_size = d_dense_qp_dim_memsize();
  dim_mem = malloc(dim_size);
  d_dense_qp_dim_create(&dims_, dim_mem);

  hpipm_size_t qp_size = d_dense_qp_memsize(&dims_);
  void *qp_mem = malloc(qp_size);
  d_dense_qp_create(&dims_, &qp, qp_mem);

  qp_sol_size = d_dense_qp_sol_memsize(&dims_);
  qp_sol_mem = malloc(qp_sol_size);
  d_dense_qp_sol_create(&dims_, &qp_sol, qp_sol_mem);

  ipm_arg_size = d_dense_qp_ipm_arg_memsize(&dims_);
  ipm_arg_mem = malloc(ipm_arg_size);

  d_dense_qp_ipm_arg_create(&dims_, &arg, ipm_arg_mem);

  d_dense_qp_ipm_arg_set_default(hpipm_mode::SPEED, &arg);

  d_dense_qp_ipm_arg_set_mu0(&settings.mu0, &arg);
  d_dense_qp_ipm_arg_set_iter_max(&settings.iter_max, &arg);
  d_dense_qp_ipm_arg_set_alpha_min(&settings.alpha_min, &arg);
  d_dense_qp_ipm_arg_set_tol_stat(&settings.tol_stat, &arg);
  d_dense_qp_ipm_arg_set_tol_eq(&settings.tol_eq, &arg);
  d_dense_qp_ipm_arg_set_tol_ineq(&settings.tol_ineq, &arg);
  d_dense_qp_ipm_arg_set_tol_comp(&settings.tol_comp, &arg);
  d_dense_qp_ipm_arg_set_reg_prim(&settings.reg_prim, &arg);
  d_dense_qp_ipm_arg_set_reg_dual(&settings.reg_dual, &arg);
  d_dense_qp_ipm_arg_set_warm_start(&settings.warm_start, &arg);
  d_dense_qp_ipm_arg_set_pred_corr(&settings.pred_corr, &arg);
  d_dense_qp_ipm_arg_set_split_step(&settings.split_step, &arg);

  ipm_size = d_dense_qp_ipm_ws_memsize(&dims_, &arg);
  ipm_mem = malloc(ipm_size);

  d_dense_qp_ipm_ws_create(&dims_, &arg, &workspace, ipm_mem);

  verbose = settings.verbose;
}

QpSolver::~QpSolver() {
  free(dim_mem);
  free(qp_mem);
  free(qp_sol_mem);
  free(ipm_arg_mem);
  free(ipm_mem);
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

  // call solver
  int hpipm_status;
  d_dense_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
  d_dense_qp_ipm_get_status(&workspace, &hpipm_status);
  this->print(hpipm_status);

  sol.setZero(dims_.nv);
  d_dense_qp_sol_get_v(&qp_sol, sol.data());

  return sol;
}

void QpSolver::print(int hpipm_status) {
  if (verbose) {
    printf("\nHPIPM returned with flag %i.\n", hpipm_status);
    if (hpipm_status == 0) {
      printf("\n -> QP solved!\n");
    } else if (hpipm_status == 1) {
      printf("\n -> Solver failed! Maximum number of iterations reached\n");
    } else if (hpipm_status == 2) {
      printf("\n -> Solver failed! Minimum step lenght reached\n");
    } else if (hpipm_status == 3) {
      printf("\n -> Solver failed! NaN in computations\n");
    } else {
      printf("\n -> Solver failed! Unknown return flag\n");
    }
    printf("\n\n");

    int iter;
    d_dense_qp_ipm_get_iter(&workspace, &iter);
    double res_stat;
    d_dense_qp_ipm_get_max_res_stat(&workspace, &res_stat);
    double res_eq;
    d_dense_qp_ipm_get_max_res_eq(&workspace, &res_eq);
    double res_ineq;
    d_dense_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
    double res_comp;
    d_dense_qp_ipm_get_max_res_comp(&workspace, &res_comp);
    double *stat;
    d_dense_qp_ipm_get_stat(&workspace, &stat);
    int stat_m;
    d_dense_qp_ipm_get_stat_m(&workspace, &stat_m);

    printf("\nipm return = %d\n", hpipm_status);
    printf(
        "\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n",
        res_stat, res_eq, res_ineq, res_comp);

    printf("\nipm iter = %d\n", iter);
    printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_"
           "stat\tres_eq\t\tres_ineq\tres_comp\tobj\t\tlq fact\t\titref "
           "pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res "
           "comp\n");
    d_print_exp_tran_mat(stat_m, iter + 1, stat, stat_m);
  }
}

} // namespace clear
