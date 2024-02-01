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
class LipGen {

public:
  LipGen(Node::SharedPtr nodeHandle,
         std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
         std::shared_ptr<ReferenceBuffer> referenceBuffer);

  ~LipGen();

  void optimize();

  void setVelCmd(vector3_t vd, scalar_t yawd);

  void setHeightCmd(scalar_t h);

private:
  void generateTrajRef();

  void getDynamics(scalar_t time_cur, size_t k,
                   const std::shared_ptr<ModeSchedule> mode_schedule);

  void
  getInequalityConstraints(size_t k, size_t N,
                           const std::shared_ptr<ModeSchedule> mode_schedule);

  void getCosts(scalar_t time_cur, size_t k, size_t N,
                const std::shared_ptr<ModeSchedule> mode_schedule);

  void fitTraj(scalar_t time_cur, size_t N);

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;
  std::shared_ptr<CubicSplineTrajectory> pos_ref_ptr_;

  std::string base_name;
  std::vector<std::string> foot_names;

  scalar_t h_des = 0.57;
  scalar_t dt_ = 0.02;
  const scalar_t grav_ = 9.81;
  matrix_t weight_;
  bool first_run_ = true;
  std::map<std::string, vector3_t>
      footholds_nominal_pos; // nominal footholds relative to nominal com pos

  std::vector<hpipm::OcpQp> ocp_;
  std::vector<hpipm::OcpQpSolution> solution_;
  hpipm::OcpQpIpmSolverSettings solver_settings;

  vector3_t vel_cmd;
  scalar_t yawd_ = 0.0;

  vector3_t pos_start;
  bool first_run = true;
  scalar_t t0 = 0.0;
};

} // namespace clear
