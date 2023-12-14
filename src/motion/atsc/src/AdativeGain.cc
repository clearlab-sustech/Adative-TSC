#include "atsc/AdativeGain.h"
#include <pinocchio/Orientation.h>
#include <rcpputils/asserts.hpp>

namespace clear {

AdaptiveGain::AdaptiveGain(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr,
    std::string base_name)
    : nodeHandle_(nodeHandle), pinocchioInterfacePtr_(pinocchioInterfacePtr),
      base_name_(base_name) {
  total_mass_ = pinocchioInterfacePtr_->total_mass();
  weight_.setZero(12, 12);
  // weight_.diagonal() << 100, 100, 100, 20.0, 20.0, 20.0, 200, 200, 200, 10.0,
  //     10.0, 10.0;

  weight_.diagonal() << 40, 40, 50, 0.3, 0.3, 0.3, 30, 30, 50, 0.2, 0.2, 0.3;
  // weight_ = 20.0 * weight_;

  solver_settings.mode = hpipm::HpipmMode::Speed;
  solver_settings.iter_max = 50;
  solver_settings.alpha_min = 1e-8;
  solver_settings.mu0 = 1e2;
  solver_settings.tol_stat = 1e-06;
  solver_settings.tol_eq = 1e-06;
  solver_settings.tol_ineq = 1e-06;
  solver_settings.tol_comp = 1e-06;
  solver_settings.reg_prim = 1e-12;
  solver_settings.pred_corr = 1;
  solver_settings.ric_alg = 0;
  solver_settings.split_step = 1;
}

AdaptiveGain::~AdaptiveGain() {}

void AdaptiveGain::update_trajectory_reference(
    std::shared_ptr<TrajectoriesArray> referenceTrajectoriesPtr) {
  refTrajBuffer_.push(referenceTrajectoriesPtr);
}

void AdaptiveGain::update_mode_schedule(
    const std::shared_ptr<ModeSchedule> mode_schedule) {
  mode_schedule_buffer.push(mode_schedule);
}

void AdaptiveGain::add_linear_system(size_t k) {
  const scalar_t time_k = nodeHandle_->now().seconds() + k * dt_;
  auto mode_schedule = mode_schedule_buffer.get();
  auto pos_traj = refTrajBuffer_.get()->get_base_pos_traj();
  auto rpy_traj = refTrajBuffer_.get()->get_base_rpy_traj();
  auto foot_traj = refTrajBuffer_.get()->get_foot_pos_traj();

  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  const size_t nf = pinocchioInterfacePtr_->getContactPoints().size();
  rcpputils::assert_true(nf == contact_flag.size());

  Ig_ = pinocchioInterfacePtr_->getData().Ig.inertia();
  std::cout << "Ig_: \n" << Ig_ << "\n";
  auto base_pose = pinocchioInterfacePtr_->getFramePose(base_name_);
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
  // vector3_t xc = pinocchioInterfacePtr_->getCoMPos();
  const auto &foot_names = pinocchioInterfacePtr_->getContactPoints();
  for (size_t i = 0; i < nf; i++) {
    if (contact_flag[i]) {
      vector3_t pf_i = foot_traj[foot_names[i]]->evaluate(time_k);
      ocp_[k].B.middleRows(3, 3).middleCols(3 * i, 3).diagonal().fill(
          dt_ / total_mass_);
      ocp_[k].B.bottomRows(3).middleCols(3 * i, 3) = skew(dt_ * (pf_i - xc));
    }
  }

  ocp_[k].b.setZero(12);
  ocp_[k].b.segment(3, 3) = -dt_ * pos_traj->derivative(time_k, 2);
  ocp_[k].b(5) += -dt_ * grav_;
  ocp_[k].b.tail(3) =
      -dt_ *
      (Ig_ * (getJacobiFromRPYToOmega(rpy) * rpy_ddot_des +
              getJacobiDotFromRPYToOmega(rpy, rpy_dot_des) * rpy_dot_des) +
       skew(rpy_dot_des) * Ig_ * rpy_dot_des);
}

void AdaptiveGain::add_state_input_constraints(size_t k, size_t N) {
  const size_t nf = pinocchioInterfacePtr_->getContactPoints().size();
  auto mode_schedule = mode_schedule_buffer.get();
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

void AdaptiveGain::add_cost(size_t k, size_t N) {
  const size_t nf = pinocchioInterfacePtr_->getContactPoints().size();
  const scalar_t time_k = time_now_ + k * dt_;
  auto pos_traj = refTrajBuffer_.get()->get_base_pos_traj();

  ocp_[k].Q = weight_;
  ocp_[k].S = matrix_t::Zero(3 * nf, 12);
  ocp_[k].q.setZero(12);
  ocp_[k].r.setZero(3 * nf);
  if (k < N) {
    ocp_[k].R = 1e-4 * matrix_t::Identity(3 * nf, 3 * nf);
    auto mode_schedule = mode_schedule_buffer.get();
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

std::shared_ptr<AdaptiveGain::FeedbackGain> AdaptiveGain::compute() {
  feedback_law_ptr = nullptr;

  auto pos_traj = refTrajBuffer_.get()->get_base_pos_traj();
  auto rpy_traj = refTrajBuffer_.get()->get_base_rpy_traj();
  if (pos_traj.get() == nullptr || rpy_traj.get() == nullptr ||
      refTrajBuffer_.get()->get_foot_pos_traj().empty()) {
    return feedback_law_ptr;
  }
  time_now_ = nodeHandle_->now().seconds();

  size_t N = pos_traj->duration() / dt_;
  ocp_.resize(N + 1);

  for (size_t k = 0; k <= N; k++) {
    if (k < N) {
      add_linear_system(k);
    }
    add_state_input_constraints(k, N);
    add_cost(k, N);
  }
  if (solution_.size() == N + 1) {
    solver_settings.warm_start = 1;
  } else {
    solver_settings.warm_start = 0;
    solution_.resize(N + 1);
  }
  hpipm::OcpQpIpmSolver solver(ocp_, solver_settings);

  vector_t x0(12);
  auto base_pose = pinocchioInterfacePtr_->getFramePose(base_name_);
  auto base_twist =
      pinocchioInterfacePtr_->getFrame6dVel_localWorldAligned(base_name_);
  vector3_t rpy = toEulerAngles(base_pose.rotation());
  vector3_t rpy_des = rpy_traj->evaluate(time_now_);
  vector3_t rpy_dot_des = rpy_traj->derivative(time_now_, 1);
  auto rpy_err = compute_euler_angle_err(rpy, rpy_des);

  x0 << base_pose.translation() - pos_traj->evaluate(time_now_),
      base_twist.linear() - pos_traj->derivative(time_now_, 1), rpy_err,
      Ig_ * base_twist.angular() -
          Ig_ * getJacobiFromRPYToOmega(rpy) * rpy_dot_des;
  const auto res = solver.solve(x0, ocp_, solution_);
  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    has_sol_ = true;
    feedback_law_ptr = std::make_shared<FeedbackGain>();
    matrix_t P(6, 12);
    P.setZero();
    P.topRows(3).middleCols(3, 3).setIdentity();
    P.bottomRows(3).middleCols(9, 3) = Ig_.inverse();

    matrix_t A = 1.0 / dt_ * (ocp_[0].A - matrix_t::Identity(12, 12));
    matrix_t B = 1.0 / dt_ * ocp_[0].B;
    vector_t drift = 1.0 / dt_ * ocp_[0].b;
    feedback_law_ptr->K = P * (A + B * solution_[0].K);
    feedback_law_ptr->b = P * (B * solution_[0].k + drift);
    vector3_t omega_des = getJacobiFromRPYToOmega(rpy) * rpy_dot_des;
    feedback_law_ptr->b.tail(3) -= Ig_.inverse() *
                                   skew(base_twist.angular() - omega_des) *
                                   Ig_ * (base_twist.angular() - omega_des);

    std::vector<scalar_t> time_array;
    std::vector<vector_t> base_pos_array;
    std::vector<vector_t> base_rpy_array;

    for (size_t k = 0; k <= N; k++) {
      time_array.emplace_back(time_now_ + k * dt_);
      base_pos_array.emplace_back(solution_[k].x.head(3));
      base_rpy_array.emplace_back(solution_[k].x.segment(6, 3));
    }
    auto base_pos_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
        3, CubicSplineInterpolation::SplineType::cspline);
    base_pos_traj_ptr_->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv,
        solution_[0].x.segment(3, 3),
        CubicSplineInterpolation::BoundaryType::first_deriv,
        solution_.back().x.segment(3, 3));
    base_pos_traj_ptr_->fit(time_array, base_pos_array);

    auto base_rpy_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
        3, CubicSplineInterpolation::SplineType::cspline_hermite);
    base_rpy_traj_ptr_->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv,
        getJacobiFromRPYToOmega(rpy) * Ig_.inverse() * solution_[0].x.tail(3),
        CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
    base_rpy_traj_ptr_->fit(time_array, base_rpy_array);

    refTrajBuffer_.get()->set_optimized_base_pos_traj(base_pos_traj_ptr_);
    refTrajBuffer_.get()->set_optimized_base_rpy_traj(base_rpy_traj_ptr_);
    /* std::cout << "#####################acc opt1######################\n"
              << (A * x0 + B * solution_[0].u).transpose()
              << "\n"; */
    // std::cout << "###########################################"
    //           << "\n";
    // for (auto &sol : solution_) {
    //   std::cout << "forward: " << sol.x.transpose() << "\n";
    // }
  } else {
    std::cout << "AdaptiveGain: " << res << "\n";
  }
  return feedback_law_ptr;
}

vector3_t AdaptiveGain::compute_euler_angle_err(const vector3_t &rpy_m,
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
