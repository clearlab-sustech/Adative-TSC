#pragma once
#include <iostream>
#include <Eigen/Sparse>
#include <core/types.h>
#include <memory>
#include <proxsuite/helpers/optional.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>

using namespace proxsuite::proxqp;
using proxsuite::nullopt;
using namespace Eigen;
using namespace std;

namespace clear {
using VectorVariables = Eigen::VectorXi;
using VariableRefList = std::list<Eigen::Ref<const VectorVariables>>;
using Triplet = Eigen::Triplet<scalar_t>;

template <typename T> class Binding {
public:
  Binding(const std::shared_ptr<T> &ptr) : ptr_(ptr) {}
  const T *Get() { return ptr_.get(); }

private:
  std::shared_ptr<T> ptr_;
};

struct BoundingBoxConstraint {
  vector_t lb, ub;
  VectorVariables vars;
};

struct LinearEqualityConstraint {
  SparseMatrix<scalar_t> Aeq;
  SparseVector<scalar_t> beq;
  VectorVariables vars;
};

struct LinearInEqualityConstraint {
  SparseMatrix<scalar_t> C;
  SparseVector<scalar_t> lb, ub;
  VectorVariables vars;
};

struct QuadraticCost {
  SparseMatrix<scalar_t> Q;
  SparseVector<scalar_t> f;
  VectorVariables vars;
};

class MathematicalProgram {
private:
  VectorVariables vars_;
  SparseMatrix<scalar_t> A_, C_, H_;
  SparseVector<scalar_t> bA_, lbC_, ubC_, f_;

  std::vector<Binding<LinearEqualityConstraint>> equality_cstrs;
  std::vector<Binding<LinearInEqualityConstraint>> inequality_cstrs;
  std::vector<Binding<BoundingBoxConstraint>> boundingbox_cstrs;
  std::vector<Binding<QuadraticCost>> costs;

  std::shared_ptr<sparse::QP<scalar_t, long long>> qpsolver_ptr;

  void parseConstraints();

  void parseCosts();

  VectorVariables concatenateVariableRefList(const VariableRefList &vars);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  MathematicalProgram();

  ~MathematicalProgram();

  VectorVariables newVectorVariables(size_t n);

  void
  addLinearEqualityConstraints(const Eigen::Ref<const matrix_t> &Aeq,
                               const Eigen::Ref<const vector_t> &beq,
                               const Eigen::Ref<const VectorVariables> &vars);

  void addLinearEqualityConstraints(const Eigen::Ref<const matrix_t> &Aeq,
                                    const Eigen::Ref<const vector_t> &beq,
                                    const VariableRefList &vars);

  void addBoundingBoxConstraints(const Eigen::Ref<const vector_t> lb,
                                 const Eigen::Ref<const vector_t> ub,
                                 const Eigen::Ref<const VectorVariables> &vars);

  void addBoundingBoxConstraints(const Eigen::Ref<const vector_t> lb,
                                 const Eigen::Ref<const vector_t> ub,
                                 const VariableRefList &vars);

  void
  addLinearInEqualityConstraints(const Eigen::Ref<const matrix_t> C,
                                 const Eigen::Ref<const vector_t> lb,
                                 const Eigen::Ref<const vector_t> ub,
                                 const Eigen::Ref<const VectorVariables> &vars);

  void addLinearInEqualityConstraints(const Eigen::Ref<const matrix_t> C,
                                      const Eigen::Ref<const vector_t> lb,
                                      const Eigen::Ref<const vector_t> ub,
                                      const VariableRefList &vars);

  void addQuadraticCost(const Eigen::Ref<const matrix_t> Q,
                        const Eigen::Ref<const vector_t> f,
                        const Eigen::Ref<const VectorVariables> &vars);

  void addQuadraticCost(const Eigen::Ref<const matrix_t> Q,
                        const Eigen::Ref<const vector_t> f,
                        const VariableRefList &vars);

  void addQuadraticErrorCost(const Eigen::Ref<const matrix_t> Q,
                             const Eigen::Ref<const vector_t> vars_des,
                             const Eigen::Ref<const VectorVariables> &vars);

  void addQuadraticErrorCost(const Eigen::Ref<const matrix_t> Q,
                             const Eigen::Ref<const vector_t> vars_des,
                             const VariableRefList &vars);

  void addQuadraticErrorCost(const Eigen::Ref<const matrix_t> Q,
                             const Eigen::Ref<const matrix_t> P,
                             const Eigen::Ref<const vector_t> vars_des,
                             const Eigen::Ref<const VectorVariables> &vars);

  void addQuadraticErrorCost(const Eigen::Ref<const matrix_t> Q,
                             const Eigen::Ref<const matrix_t> P,
                             const Eigen::Ref<const vector_t> vars_des,
                             const VariableRefList &vars);

  bool solve();

  vector_t getSolution();

  vector_t getSolution(const Eigen::Ref<const VectorVariables> &vars);

  vector_t getSolution(const VariableRefList &vars);
};

} // namespace clear