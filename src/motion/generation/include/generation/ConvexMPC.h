#pragma once
#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>
#include <asserts/trajectory/TrajectoriesArray.h>

#include <core/misc/Buffer.h>
#include <hpipm-cpp/hpipm-cpp.hpp>
#include <pinocchio/Orientation.h>
#include <pinocchio/PinocchioInterface.h>

namespace clear {
class ConvexMPC {
public:
  ConvexMPC(PinocchioInterface &pinocchioInterface,
            std::shared_ptr<TrajectoriesArray> referenceTrajectoriesBuffer);

  ~ConvexMPC();

  void optimize(scalar_t time_cur,
                const std::shared_ptr<ModeSchedule> mode_schedule);

  std::shared_ptr<CubicSplineTrajectory> get_base_pos_trajectory();

  std::shared_ptr<CubicSplineTrajectory> get_base_rpy_trajectory();

private:
  void get_dynamics(scalar_t time_cur, size_t k,
                    const std::shared_ptr<ModeSchedule> mode_schedule);

  void
  get_inequality_constraints(size_t k, size_t N,
                             const std::shared_ptr<ModeSchedule> mode_schedule);

  void get_costs(scalar_t time_cur, size_t k, size_t N,
                 const std::shared_ptr<ModeSchedule> mode_schedule);

  void fit_traj(scalar_t time_cur, size_t N);

  vector3_t compute_euler_angle_err(const vector3_t &rpy_m,
                                    const vector3_t &rpy_d);

private:
  PinocchioInterface &pinocchioInterface_;
  std::shared_ptr<TrajectoriesArray> referenceTrajectoriesBuffer_;

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
