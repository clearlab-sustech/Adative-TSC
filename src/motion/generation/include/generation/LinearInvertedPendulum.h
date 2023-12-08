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

  void optimize(std::shared_ptr<CubicSplineTrajectory> pos_ref);

  std::map<std::string, std::pair<scalar_t, vector3_t>> get_footholds();

private:
  void get_dynamics(size_t k,
                    const std::shared_ptr<ModeSchedule> mode_schedule);

  void
  get_inequality_constraints(scalar_t time_cur, size_t k, size_t N,
                             std::shared_ptr<CubicSplineTrajectory> pos_ref,
                             const std::shared_ptr<ModeSchedule> mode_schedule);

  void get_costs(scalar_t time_cur, size_t k, size_t N,
                 std::shared_ptr<CubicSplineTrajectory> pos_ref);

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
  std::map<std::string, std::pair<scalar_t, vector3_t>> footholds_; // footholds
  std::map<std::string, vector3_t>
      footholds_nominal_pos; // nominal footholds relative to nominal com pos
};

} // namespace clear
