#pragma once
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <core/trajectory/ReferenceBuffer.h>
#include <hpipm-cpp/hpipm-cpp.hpp>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_mpc/SystemObservation.h>

using namespace rclcpp;

namespace clear {
class ConstructVectorField {
public:
  struct VectorFieldParam {
    matrix_t K;
    vector_t b;
    vector_t force_des;
  };

public:
  ConstructVectorField(
      Node::SharedPtr nodeHandle,
      std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr);

  ~ConstructVectorField();

  void updateReferenceBuffer(std::shared_ptr<ReferenceBuffer> referenceBuffer);

  void updateMpcSol(std::shared_ptr<ocs2::PrimalSolution> mpc_sol);

  std::shared_ptr<VectorFieldParam> compute();

private:
  void add_linear_system(size_t k);

  void add_state_input_constraints(size_t k, size_t N);

  void add_cost(size_t k, size_t N);

  vector3_t computeEulerAngleErr(const vector3_t &rpy_m,
                                 const vector3_t &rpy_d);

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  std::string base_name;
  std::vector<std::string> foot_names;
  
  Buffer<std::shared_ptr<ocs2::PrimalSolution>> mpc_sol_buffer;
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;
  std::shared_ptr<VectorFieldParam> feedback_law_ptr;
  const scalar_t dt_ = 0.02;
  const scalar_t grav_ = 9.81;
  scalar_t total_mass_;
  matrix3_t Ig_;
  const scalar_t mu_ = 0.5;
  matrix_t weight_;
  scalar_t time_now_;
  vector3_t rpy_des_start;

  std::vector<hpipm::OcpQp> ocp_;
  std::vector<hpipm::OcpQpSolution> solution_;
  hpipm::OcpQpIpmSolverSettings solver_settings;
  bool has_sol_ = false;
};

} // namespace clear
