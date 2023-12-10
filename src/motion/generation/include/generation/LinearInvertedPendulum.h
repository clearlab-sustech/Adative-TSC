#pragma once
#include <asserts/trajectory/ReferenceBuffer.h>
#include <core/misc/Buffer.h>
#include <hpipm-cpp/hpipm-cpp.hpp>
#include <pinocchio/Orientation.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;

namespace clear {
class LinearInvertedPendulum {
public:
  LinearInvertedPendulum(
      Node::SharedPtr nodeHandle,
      std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
      std::shared_ptr<ReferenceBuffer> referenceBuffer);

  ~LinearInvertedPendulum();

  void optimize();

private:
  void get_dynamics(size_t k,
                    const std::shared_ptr<ModeSchedule> mode_schedule);

  void get_inequality_constraints(size_t k);

  void get_costs(scalar_t time_cur, size_t k, size_t N,
                 const std::shared_ptr<ModeSchedule> mode_schedule);

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;

  const scalar_t dt_ = 0.04;
  const scalar_t grav_ = 9.81;
  scalar_t nominal_z_;
  matrix_t weight_;

  std::vector<hpipm::OcpQp> ocp_;
  std::vector<hpipm::OcpQpSolution> solution_;
  hpipm::OcpQpIpmSolverSettings solver_settings;
  bool has_sol_ = false;
  std::string base_name;
  std::vector<string> foot_names;
};

} // namespace clear
