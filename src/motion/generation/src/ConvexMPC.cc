#include "generation/ConvexMPC.h"
#include <rcpputils/asserts.hpp>

namespace clear {

ConvexMPC::ConvexMPC(
    PinocchioInterface &pinocchioInterface,
    std::shared_ptr<TrajectoriesArray> referenceTrajectoriesBuffer)
    : pinocchioInterface_(pinocchioInterface),
      referenceTrajectoriesBuffer_(referenceTrajectoriesBuffer) {
  total_mass_ = pinocchioInterface_.total_mass();
  weight_.setZero(12, 12);
  weight_.diagonal() << 100, 100, 100, 20.0, 20.0, 20.0, 200, 200, 200, 40.0,
      40.0, 40.0;

  solver_settings.mode = hpipm::HpipmMode::Speed;
  solver_settings.iter_max = 30;
  solver_settings.alpha_min = 1e-8;
  solver_settings.mu0 = 1e2;
  solver_settings.tol_stat = 1e-04;
  solver_settings.tol_eq = 1e-04;
  solver_settings.tol_ineq = 1e-04;
  solver_settings.tol_comp = 1e-04;
  solver_settings.reg_prim = 1e-12;
  solver_settings.pred_corr = 1;
  solver_settings.ric_alg = 0;
  solver_settings.split_step = 1;
}

ConvexMPC::~ConvexMPC() {}

void ConvexMPC::get_dynamics(
    scalar_t time_cur, size_t k,
    const std::shared_ptr<ModeSchedule> mode_schedule) {
  const scalar_t time_k = time_cur + k * dt_;
  auto pos_traj = referenceTrajectoriesBuffer_->get_base_pos_ref_traj();
  auto rpy_traj = referenceTrajectoriesBuffer_->get_base_rpy_traj();
  auto foot_traj = referenceTrajectoriesBuffer_->get_foot_pos_traj();

  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  auto foot_names = pinocchioInterface_.getContactPoints();
  const size_t nf = foot_names.size();
  rcpputils::assert_true(nf == contact_flag.size());

  auto base_pose = pinocchioInterface_.getFramePose("trunk");
  vector3_t rpy = toEulerAngles(base_pose.rotation());
  vector3_t force_ff = vector3_t::Zero();
  if (has_sol_ &&
      k < solution_.size() - 1) { // the last one solution has no data of u
    for (size_t i = 0; i < nf; i++) {
      force_ff += solution_[k].u.segment(3 * i, 3);
    }
  } else {
    force_ff = total_mass_ *
               (pos_traj->derivative(time_k, 2) + vector3_t(0, 0, grav_));
  }

  vector3_t rpy_dot_des = rpy_traj->derivative(time_k, 1);
  vector3_t rpy_ddot_des = rpy_traj->derivative(time_k, 2);

  ocp_[k].A.setIdentity(12, 12);
  ocp_[k].A.block<3, 3>(0, 3).diagonal().fill(dt_);
  ocp_[k].A.block<3, 3>(6, 9) =
      dt_ * getJacobiFromOmegaToRPY(rpy) * Ig_.inverse();
  ocp_[k].A.block<3, 3>(9, 0) = skew(dt_ * force_ff);
  ocp_[k].B.setZero(12, nf * 3);
  vector3_t xc = pos_traj->evaluate(time_k);
  for (size_t i = 0; i < nf; i++) {
    const auto &foot_name = foot_names[i];
    if (contact_flag[i]) {
      vector3_t pf_i = foot_traj[foot_name]->evaluate(time_k);
      ocp_[k].B.middleRows(3, 3).middleCols(3 * i, 3).diagonal().fill(
          dt_ / total_mass_);
      ocp_[k].B.bottomRows(3).middleCols(3 * i, 3) = skew(dt_ * (pf_i - xc));
    }
  }

  ocp_[k].b.setZero(12);
  ocp_[k].b(5) += -dt_ * grav_;
  ocp_[k].b.tail(3) =
      -dt_ *
          (Ig_ * (getJacobiFromRPYToOmega(rpy) * rpy_ddot_des +
                  getJacobiDotFromRPYToOmega(rpy, rpy_dot_des) * rpy_dot_des) +
           skew(rpy_dot_des) * Ig_ * rpy_dot_des) -
      skew(dt_ * force_ff) * xc;
}

void ConvexMPC::get_inequality_constraints(
    size_t k, size_t N, const std::shared_ptr<ModeSchedule> mode_schedule) {
  auto foot_names = pinocchioInterface_.getContactPoints();
  const size_t nf = foot_names.size();
  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));

  if (k < N) {
    scalar_t mu = 1 / mu_;
    matrix_t Ci(5, 3);
    Ci << mu, 0, 1., -mu, 0, 1., 0, mu, 1., 0, -mu, 1., 0, 0, 1.;
    ocp_[k].C = matrix_t::Zero(5 * nf, 12);
    ocp_[k].D.setZero(5 * nf, 3 * nf);
    ocp_[k].lg.setZero(5 * nf);
    ocp_[k].ug.setZero(5 * nf);
    ocp_[k].lg_mask.setOnes(5 * nf);
    ocp_[k].ug_mask.setOnes(5 * nf);

    for (size_t i = 0; i < nf; i++) {
      ocp_[k].D.block<5, 3>(i * 5, i * 3) = Ci;
      ocp_[k].ug(5 * i + 4) = contact_flag[i] ? 400 : 0.0;
      ocp_[k].ug_mask.segment(5 * i, 4).setZero();
    }
  }
  /* std::cout << "\n############### " << k << " constraints ################\n"
           << cstr_k; */
}

void ConvexMPC::get_costs(scalar_t time_cur, size_t k, size_t N,
                           const std::shared_ptr<ModeSchedule> mode_schedule) {
  auto foot_names = pinocchioInterface_.getContactPoints();
  const size_t nf = foot_names.size();
  const scalar_t time_k = time_cur + k * dt_;
  auto pos_traj = referenceTrajectoriesBuffer_->get_base_pos_ref_traj();
  auto rpy_traj = referenceTrajectoriesBuffer_->get_base_rpy_traj();

  vector3_t rpy_des =
      rpy_traj->evaluate(time_k) - rpy_traj->evaluate(time_cur) + rpy_des_start;
  vector3_t omega_des =
      getJacobiFromRPYToOmega(rpy_des) * rpy_traj->derivative(time_k, 1);

  vector_t x_des(12);
  x_des << pos_traj->evaluate(time_k), pos_traj->derivative(time_k, 1), rpy_des,
      omega_des;

  ocp_[k].Q = weight_;
  ocp_[k].S = matrix_t::Zero(3 * nf, 12);
  ocp_[k].q = -weight_ * x_des;
  ocp_[k].r.setZero(3 * nf);
  if (k < N) {
    ocp_[k].R = 1e-4 * matrix_t::Identity(3 * nf, 3 * nf);
    scalar_t phase = k * dt_ / mode_schedule->duration();
    auto contact_flag =
        quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
    int nc = 0;
    for (bool flag : contact_flag) {
      if (flag) {
        nc++;
      }
    }
    nc = max(1, nc);
    vector3_t force_des_i =
        total_mass_ / nc *
        (vector3_t(0, 0, grav_) + pos_traj->derivative(time_k, 2));
    vector_t force_des = vector_t::Zero(3 * nf);
    for (size_t k = 0; k < nf; k++) {
      if (contact_flag[k]) {
        force_des.segment(3 * k, 3) = force_des_i;
      }
    }
    ocp_[k].r = -ocp_[k].R * force_des;
  } else {
    ocp_[k].Q = 1e2 * ocp_[k].Q;
    ocp_[k].q = 1e2 * ocp_[k].q;
  }
}

void ConvexMPC::optimize(scalar_t time_cur,
                          const std::shared_ptr<ModeSchedule> mode_schedule) {
  auto pos_traj = referenceTrajectoriesBuffer_->get_base_pos_ref_traj();
  auto rpy_traj = referenceTrajectoriesBuffer_->get_base_rpy_traj();
  if (pos_traj.get() == nullptr || rpy_traj.get() == nullptr ||
      referenceTrajectoriesBuffer_->get_foot_pos_traj().empty()) {
    return;
  }

  size_t N = pos_traj->duration() / dt_;
  ocp_.resize(N + 1);
  Ig_ = pinocchioInterface_.getData().Ig.inertia();
  auto base_pose = pinocchioInterface_.getFramePose("trunk");
  auto base_twist =
      pinocchioInterface_.getFrame6dVel_localWorldAligned("trunk");
  auto rpy_m = toEulerAngles(base_pose.rotation());
  rpy_des_start =
      rpy_m - compute_euler_angle_err(rpy_m, rpy_traj->evaluate(time_cur));

  for (size_t k = 0; k <= N; k++) {
    if (k < N) {
      get_dynamics(time_cur, k, mode_schedule);
    }
    get_inequality_constraints(k, N, mode_schedule);
    get_costs(time_cur, k, N, mode_schedule);
  }

  if (solution_.size() == N + 1) {
    solver_settings.warm_start = 1;
  } else {
    solver_settings.warm_start = 0;
    solution_.resize(N + 1);
  }
  hpipm::OcpQpIpmSolver solver(ocp_, solver_settings);

  vector_t x0(12);
  x0 << base_pose.translation(), base_twist.linear(), rpy_m,
      Ig_ * base_twist.angular();

  const auto res = solver.solve(x0, ocp_, solution_);
  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    has_sol_ = true;
    fit_traj(time_cur, N);
  } else {
    std::cout << "ConvexMPC: " << res << "\n";
    exit(0);
  }
}

void ConvexMPC::fit_traj(scalar_t time_cur, size_t N) {
  std::vector<scalar_t> time_array;
  std::vector<vector_t> base_pos_array;
  std::vector<vector_t> base_rpy_array;

  for (size_t k = 1; k <= N; k++) {
    time_array.emplace_back(time_cur + k * dt_);
    base_pos_array.emplace_back(solution_[k].x.head(3));
    base_rpy_array.emplace_back(solution_[k].x.segment(6, 3));
  }
  base_pos_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_pos_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      solution_[1].x.segment(3, 3),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      solution_.back().x.segment(3, 3));
  base_pos_traj_ptr_->fit(time_array, base_pos_array);

  base_rpy_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline_hermite);
  base_rpy_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
  base_rpy_traj_ptr_->fit(time_array, base_rpy_array);
}

std::shared_ptr<CubicSplineTrajectory> ConvexMPC::get_base_pos_trajectory() {
  return base_pos_traj_ptr_;
}

std::shared_ptr<CubicSplineTrajectory> ConvexMPC::get_base_rpy_trajectory() {
  return base_rpy_traj_ptr_;
}

vector3_t ConvexMPC::compute_euler_angle_err(const vector3_t &rpy_m,
                                              const vector3_t &rpy_d) {
  vector3_t rpy_err = rpy_m - rpy_d;
  if (rpy_err.norm() > 1.5 * M_PI) {
    if (abs(rpy_err(0)) > M_PI) {
      rpy_err(0) += (rpy_err(0) > 0 ? -2.0 : 2.0) * M_PI;
    }
    if (abs(rpy_err(1)) > M_PI) {
      rpy_err(1) += (rpy_err(1) > 0 ? -2.0 : 2.0) * M_PI;
    }
    if (abs(rpy_err(2)) > M_PI) {
      rpy_err(2) += (rpy_err(2) > 0 ? -2.0 : 2.0) * M_PI;
    }
  }
  return rpy_err;
}

} // namespace clear
