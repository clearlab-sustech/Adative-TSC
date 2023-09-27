#include "core/optimization/MathematicalProgram.h"

#include <iostream>

namespace clear {
MathematicalProgram::MathematicalProgram() {}

MathematicalProgram::~MathematicalProgram() {}

VectorVariables MathematicalProgram::newVectorVariables(size_t n) {
  VectorVariables var_new(n);
  var_new = VectorVariables::LinSpaced(n, vars_.size(), vars_.size() + n);
  vars_ = VectorVariables::LinSpaced(vars_.size() + n, 0, vars_.size() + n);
  return var_new;
}

void MathematicalProgram::addLinearEqualityConstraints(
    const Eigen::Ref<const matrix_t> &Aeq,
    const Eigen::Ref<const vector_t> &beq,
    const Eigen::Ref<const VectorVariables> &vars) {
  auto cstr = std::make_shared<LinearEqualityConstraint>();
  cstr->Aeq = Aeq.sparseView();
  cstr->beq = beq.sparseView();
  cstr->vars = vars;
  equality_cstrs.push_back(Binding<LinearEqualityConstraint>(cstr));
}

void MathematicalProgram::addLinearEqualityConstraints(
    const Eigen::Ref<const matrix_t> &Aeq,
    const Eigen::Ref<const vector_t> &beq, const VariableRefList &vars) {
  addLinearEqualityConstraints(Aeq, beq, concatenateVariableRefList(vars));
}

void MathematicalProgram::addBoundingBoxConstraints(
    const Eigen::Ref<const vector_t> lb, const Eigen::Ref<const vector_t> ub,
    const Eigen::Ref<const VectorVariables> &vars) {
  auto cstr = std::make_shared<BoundingBoxConstraint>();
  cstr->lb = lb;
  cstr->ub = ub;
  cstr->vars = vars;
  boundingbox_cstrs.push_back(Binding<BoundingBoxConstraint>(cstr));
}

void MathematicalProgram::addBoundingBoxConstraints(
    const Eigen::Ref<const vector_t> lb, const Eigen::Ref<const vector_t> ub,
    const VariableRefList &vars) {
  addBoundingBoxConstraints(lb, ub, concatenateVariableRefList(vars));
}

void MathematicalProgram::addLinearInEqualityConstraints(
    const Eigen::Ref<const matrix_t> C, const Eigen::Ref<const vector_t> lb,
    const Eigen::Ref<const vector_t> ub,
    const Eigen::Ref<const VectorVariables> &vars) {
  auto cstr = std::make_shared<LinearInEqualityConstraint>();
  cstr->C = C.sparseView();
  cstr->lb = lb.sparseView();
  cstr->ub = ub.sparseView();
  cstr->vars = vars;
  inequality_cstrs.push_back(Binding<LinearInEqualityConstraint>(cstr));
}

void MathematicalProgram::addLinearInEqualityConstraints(
    const Eigen::Ref<const matrix_t> C, const Eigen::Ref<const vector_t> lb,
    const Eigen::Ref<const vector_t> ub, const VariableRefList &vars) {
  addLinearInEqualityConstraints(C, lb, ub, concatenateVariableRefList(vars));
}

void MathematicalProgram::addQuadraticCost(
    const Eigen::Ref<const matrix_t> Q, const Eigen::Ref<const vector_t> f,
    const Eigen::Ref<const VectorVariables> &vars) {
  auto cost = std::make_shared<QuadraticCost>();
  cost->Q = Q.sparseView();
  cost->f = f.sparseView();
  cost->vars = vars;
  costs.push_back(Binding<QuadraticCost>(cost));
}

void MathematicalProgram::addQuadraticCost(const Eigen::Ref<const matrix_t> Q,
                                           const Eigen::Ref<const vector_t> f,
                                           const VariableRefList &vars) {
  addQuadraticCost(Q, f, concatenateVariableRefList(vars));
}

void MathematicalProgram::addQuadraticErrorCost(
    const Eigen::Ref<const matrix_t> Q,
    const Eigen::Ref<const vector_t> vars_des,
    const Eigen::Ref<const VectorVariables> &vars) {
  auto cost = std::make_shared<QuadraticCost>();
  cost->Q = Q.sparseView();
  cost->f = (-Q.transpose() * vars_des).sparseView();
  cost->vars = vars;
  costs.push_back(Binding<QuadraticCost>(cost));
}

void MathematicalProgram::addQuadraticErrorCost(
    const Eigen::Ref<const matrix_t> Q,
    const Eigen::Ref<const vector_t> vars_des, const VariableRefList &vars) {
  addQuadraticErrorCost(Q, vars_des, concatenateVariableRefList(vars));
}

void MathematicalProgram::addQuadraticErrorCost(
    const Eigen::Ref<const matrix_t> Q, const Eigen::Ref<const matrix_t> P,
    const Eigen::Ref<const vector_t> vars_des,
    const Eigen::Ref<const VectorVariables> &vars) {
  auto cost = std::make_shared<QuadraticCost>();
  matrix_t M = P.transpose() * Q;
  cost->Q = (M * P).sparseView();
  cost->f = (-M * vars_des).sparseView();
  cost->vars = vars;
  costs.push_back(Binding<QuadraticCost>(cost));
}

void MathematicalProgram::addQuadraticErrorCost(
    const Eigen::Ref<const matrix_t> Q, const Eigen::Ref<const matrix_t> P,
    const Eigen::Ref<const vector_t> vars_des, const VariableRefList &vars) {
  addQuadraticErrorCost(Q, P, vars_des, concatenateVariableRefList(vars));
}

bool MathematicalProgram::solve() {
  parseConstraints();
  parseCosts();

  qpsolver_ptr = std::make_shared<sparse::QP<scalar_t, long long>>(
      vars_.size(), A_.rows(), C_.rows());
  qpsolver_ptr->settings.max_iter = 1000;
  qpsolver_ptr->settings.eps_abs = 1e-3;
  qpsolver_ptr->settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;

  qpsolver_ptr->settings.verbose = false;
  qpsolver_ptr->init(H_, f_.toDense(), A_, bA_.toDense(), C_, lbC_.toDense(),
                     ubC_.toDense());
  qpsolver_ptr->solve();

  if (qpsolver_ptr->results.info.status == QPSolverOutput::PROXQP_SOLVED) {
    return true;
  } else {
    switch (qpsolver_ptr->results.info.status) {
    case QPSolverOutput::PROXQP_MAX_ITER_REACHED:
      std::cerr << "MathematicalProgram: the maximum number of iterations has been reached\n";
      break;
    case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
      std::cerr << "MathematicalProgram: the problem is primal infeasible\n";
      break;
    case QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
      std::cerr << "MathematicalProgram: the problem is dual infeasible\n";
      break;
    case QPSolverOutput::PROXQP_NOT_RUN:
      std::cerr << "MathematicalProgram: the solver has not been run yet\n";
      break;
    default:
      break;
    }
    return false;
  }
}

vector_t MathematicalProgram::getSolution() {
  if (qpsolver_ptr->results.info.status == QPSolverOutput::PROXQP_SOLVED) {
    return qpsolver_ptr->results.x;
  } else {
    throw runtime_error("qp has not been solved, no solution can be returned");
  }
}

vector_t MathematicalProgram::getSolution(
    const Eigen::Ref<const VectorVariables> &vars) {
  if (qpsolver_ptr->results.info.status == QPSolverOutput::PROXQP_SOLVED) {
    vector_t sol_slice(vars.size());
    for (int i = 0; i < vars.size(); i++) {
      sol_slice(i) = qpsolver_ptr->results.x(vars(i));
    }
    return sol_slice;
  } else {
    throw runtime_error("qp has not been solved, no solution can be returned");
  }
}

vector_t MathematicalProgram::getSolution(const VariableRefList &vars) {
  return getSolution(concatenateVariableRefList(vars));
}

void MathematicalProgram::parseConstraints() {
  int ne = 0;
  for (auto &eq : equality_cstrs) {
    ne += eq.Get()->Aeq.rows();
  }
  A_.resize(ne, vars_.size());
  bA_.resize(ne, 1);

  int index = 0;
  for (auto &eq : equality_cstrs) {
    for (int k = 0; k < eq.Get()->Aeq.outerSize(); ++k) {
      for (SparseMatrix<scalar_t>::InnerIterator it(eq.Get()->Aeq, k); it;
           ++it) {
        A_.coeffRef(index + it.row(), eq.Get()->vars(it.col())) = it.value();
      }
    }
    for (SparseVector<scalar_t>::InnerIterator it(eq.Get()->beq); it; ++it) {
      bA_.coeffRef(index + it.index()) = it.value();
    }
    index += eq.Get()->Aeq.rows();
  }

  int nc = 0;
  for (auto &ineq : inequality_cstrs) {
    nc += ineq.Get()->C.rows();
  }
  for (auto &bc : boundingbox_cstrs) {
    nc += bc.Get()->vars.size();
  }
  C_.resize(nc, vars_.size());
  lbC_.resize(nc, 1);
  ubC_.resize(nc, 1);
  index = 0;
  for (auto &ineq : inequality_cstrs) {
    for (int k = 0; k < ineq.Get()->C.outerSize(); ++k) {
      for (SparseMatrix<scalar_t>::InnerIterator it(ineq.Get()->C, k); it;
           ++it) {
        C_.coeffRef(index + it.row(), ineq.Get()->vars(it.col())) = it.value();
      }
    }
    for (SparseVector<scalar_t>::InnerIterator it(ineq.Get()->lb); it; ++it) {
      lbC_.coeffRef(index + it.index()) = it.value();
    }
    for (SparseVector<scalar_t>::InnerIterator it(ineq.Get()->ub); it; ++it) {
      ubC_.coeffRef(index + it.index()) = it.value();
    }
    index += ineq.Get()->C.rows();
  }

  for (auto &bc : boundingbox_cstrs) {
    for (int i = 0; i < bc.Get()->vars.size(); i++) {
      C_.coeffRef(index + i, bc.Get()->vars(i)) = 1.0;
      lbC_.coeffRef(index + i) = bc.Get()->lb(i);
      ubC_.coeffRef(index + i) = bc.Get()->ub(i);
    }
    index += bc.Get()->vars.size();
  }
}

void MathematicalProgram::parseCosts() {
  H_.resize(vars_.size(), vars_.size());
  f_.resize(vars_.size(), 1);
  for (auto &cost : costs) {
    for (int k = 0; k < cost.Get()->Q.outerSize(); ++k) {
      for (SparseMatrix<scalar_t>::InnerIterator it(cost.Get()->Q, k); it;
           ++it) {
        H_.coeffRef(cost.Get()->vars(it.row()), cost.Get()->vars(it.col())) +=
            it.value();
      }
    }

    for (SparseVector<scalar_t>::InnerIterator it(cost.Get()->f); it; ++it) {
      f_.coeffRef(cost.Get()->vars(it.index()), 1) += it.value();
    }
  }
}

VectorVariables
MathematicalProgram::concatenateVariableRefList(const VariableRefList &vars) {
  int dim = 0;
  for (const auto &var : vars) {
    dim += var.size();
  }
  VectorVariables stacked_var(dim);
  int var_count = 0;
  for (const auto &var : vars) {
    stacked_var.segment(var_count, var.rows()) = var;
    var_count += var.rows();
  }
  return stacked_var;
}

} // namespace clear