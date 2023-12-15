#pragma once
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/trajectory/ReferenceBuffer.h>

#include <core/misc/Buffer.h>
#include <hpipm-cpp/hpipm-cpp.hpp>
#include <pinocchio/Orientation.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;

namespace clear {
class ConvexMPC {
public:
  ConvexMPC(Node::SharedPtr nodeHandle,
            std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
            std::shared_ptr<ReferenceBuffer> referenceBuffer);

  ~ConvexMPC();

  void optimize();

private:
  void getDynamics(scalar_t time_cur, size_t k,
                   const std::shared_ptr<ModeSchedule> mode_schedule);

  void
  getInequalityConstraints(size_t k, size_t N,
                           const std::shared_ptr<ModeSchedule> mode_schedule);

  void getCosts(scalar_t time_cur, size_t k, size_t N,
                const std::shared_ptr<ModeSchedule> mode_schedule);

  void fitTraj(scalar_t time_cur, size_t N);

  vector3_t computeEulerAngleErr(const vector3_t &rpy_m,
                                 const vector3_t &rpy_d);

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;

  const scalar_t dt_ = 0.02;
  const scalar_t grav_ = 9.81;
  scalar_t total_mass_;
  matrix3_t Ig_;
  const scalar_t mu_ = 0.5;
  matrix_t weight_;
  vector3_t rpy_des_start;

  std::vector<hpipm::OcpQp> ocp_;
  std::vector<hpipm::OcpQpSolution> solution_;
  hpipm::OcpQpIpmSolverSettings solver_settings;
  bool has_sol_ = false;

  std::shared_ptr<CubicSplineTrajectory> base_pos_traj_ptr_;
  std::shared_ptr<CubicSplineTrajectory> base_rpy_traj_ptr_;
};

} // namespace clear
