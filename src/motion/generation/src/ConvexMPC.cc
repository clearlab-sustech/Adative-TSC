#include "generation/ConvexMPC.h"
#include <rcpputils/asserts.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

ConvexMPC::ConvexMPC(Node::SharedPtr nodeHandle,
                     std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
                     std::shared_ptr<ReferenceBuffer> referenceBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      referenceBuffer_(referenceBuffer) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  base_name = config_["model"]["base_name"].as<std::string>();

  total_mass_ = pinocchioInterface_ptr_->total_mass();
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

  vel_cmd.setZero();
  yawd_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"), "ConvexMPC: Construction done");
}

ConvexMPC::~ConvexMPC() {}

void ConvexMPC::setVelCmd(vector3_t vd, scalar_t yawd) {
  vel_cmd = vd;
  yawd_ = yawd;
}

void ConvexMPC::generateTrajRef() {
  const scalar_t t_now = nodeHandle_->now().seconds();
  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);
  if (first_run) {
    pos_start = base_pose_m.translation();
    rpy_start = toEulerAngles(base_pose_m.rotation());
    rpy_start.head(2).setZero();
    first_run = false;
  } else {
    pos_start += dt_ * vel_cmd;
    rpy_start.z() = dt_ * yawd_;

    vector_t rpy_c = toEulerAngles(base_pose_m.rotation());
    if ((rpy_c - rpy_start).norm() > 0.05) {
      rpy_start = 0.05 * (rpy_start - rpy_c).normalized() + rpy_c;
    }
    rpy_start.head(2).setZero();

    vector_t pos_c = base_pose_m.translation();
    if ((pos_c - pos_start).norm() > 0.03) {
      pos_start = 0.03 * (pos_start - pos_c).normalized() + pos_c;
    }
    pos_start.z() = h_des;
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> rpy_array, pos_array;
  scalar_t horizon_time = referenceBuffer_->getModeSchedule()->duration();

  size_t N = horizon_time / 0.05;
  vector3_t vel_des = base_pose_m.rotation() * vel_cmd;
  for (size_t k = 0; k < N; k++) {
    time.push_back(t_now + dt_ * k);
    vector3_t rpy_t = rpy_start + k * dt_ * yawd_ * vector3_t::UnitZ();
    rpy_t.head(2).setZero();
    rpy_array.emplace_back(rpy_t);

    vector3_t pos_t = pos_start + k * dt_ * vel_des;
    pos_t.z() = h_des;
    pos_array.emplace_back(pos_t);
  }

  /* if (vel_cmd.norm() < 0.05) {
    vector3_t foot_center = vector3_t::Zero();
    for (size_t k = 0; k < foot_names.size(); k++) {
      foot_center +=
          pinocchioInterface_ptr_->getFramePose(foot_names[k]).translation();
    }
    foot_center = 1.0 / static_cast<scalar_t>(foot_names.size()) * foot_center;

    scalar_t zd = 0.32;
    scalar_t mod_z = 0.0;
    time.emplace_back(t_now);
    time.emplace_back(t_now + 0.5 * horizon_time);
    time.emplace_back(t_now + horizon_time);
    rpy_array.emplace_back(vector3_t(0, 0, 0));
    rpy_array.emplace_back(vector3_t(0, 0, 0));
    rpy_array.emplace_back(vector3_t(0, 0, 0));
    pos_array.emplace_back(vector3_t(foot_center.x(), foot_center.y(),
                                     zd - mod_z * (0.05 * sin(6 * t_now))));
    pos_array.emplace_back(
        vector3_t(foot_center.x(), foot_center.y(),
                  zd - mod_z * (0.05 * sin(6 * (t_now + 0.5 * horizon_time)))));
    pos_array.emplace_back(
        vector3_t(foot_center.x(), foot_center.y(),
                  zd - mod_z * (0.05 * sin(6 * (t_now + horizon_time)))));
  } */

  /* std::cout << "############# "
            << "base traj des"
            << " ##############\n";
  for (size_t i = 0; i < time.size(); i++) {
    std::cout << " t: " << time[i] - time.front()
              << " pos: " << pos_array[i].transpose() << "\n";
  } */

  auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_pos->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv, vel_cmd,
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_pos->fit(time, pos_array);
  referenceBuffer_->setIntegratedBasePosTraj(cubicspline_pos);

  /* if (referenceBuffer_->get_base_pos_traj().get() == nullptr) {
    referenceBuffer_->set_base_pos_traj(cubicspline_pos);
  } */

  auto cubicspline_rpy = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_rpy->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector3_t(0, 0, yawd_),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_rpy->fit(time, rpy_array);
  referenceBuffer_->setIntegratedBaseRpyTraj(cubicspline_rpy);
}

void ConvexMPC::getDynamics(scalar_t time_cur, size_t k,
                            const std::shared_ptr<ModeSchedule> mode_schedule) {
  const scalar_t time_k = time_cur + k * dt_;
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto foot_traj = referenceBuffer_->getFootPosTraj();

  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  const size_t nf = foot_names.size();
  rcpputils::assert_true(nf == contact_flag.size());

  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  vector3_t rpy = toEulerAngles(base_pose.rotation());

  matrix_t Rt = toRotationMatrix(rpy_traj->evaluate(time_k));
  matrix_t Ig_t = Rt * Ig_ * Rt.transpose();

  ocp_[k].A.setIdentity(12, 12);
  ocp_[k].A.block<3, 3>(0, 3).diagonal().fill(dt_);
  ocp_[k].A.block<3, 3>(6, 9) =
      dt_ * getJacobiFromOmegaToRPY(rpy) * Ig_t.inverse();
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
}

void ConvexMPC::getInequalityConstraints(
    size_t k, size_t N, const std::shared_ptr<ModeSchedule> mode_schedule) {
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

void ConvexMPC::getCosts(scalar_t time_cur, size_t k, size_t N,
                         const std::shared_ptr<ModeSchedule> mode_schedule) {
  const size_t nf = foot_names.size();
  const scalar_t time_k = time_cur + k * dt_;
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();

  vector3_t rpy_des =
      rpy_traj->evaluate(time_k) - rpy_traj->evaluate(time_cur) + rpy_des_start;
  vector3_t omega_des =
      getJacobiFromRPYToOmega(rpy_des) * rpy_traj->derivative(time_k, 1);

  matrix_t Rt = toRotationMatrix(rpy_des);
  matrix_t Ig_t = Rt * Ig_ * Rt.transpose();

  vector_t x_des(12);
  x_des << pos_traj->evaluate(time_k), pos_traj->derivative(time_k, 1), rpy_des,
      Ig_t * omega_des;

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

void ConvexMPC::optimize() {
  RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"),
              "ConvexMPC: generateTrajRef start");
  generateTrajRef();

  const scalar_t time_cur = nodeHandle_->now().seconds();
  auto mode_schedule = referenceBuffer_->getModeSchedule();

  if (referenceBuffer_->getFootPosTraj().empty() || mode_schedule == nullptr) {
    return;
  }

  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();

  size_t N = mode_schedule->duration() / dt_;
  ocp_.resize(N + 1);

  matrix_t Ig_0 = pinocchioInterface_ptr_->getData().Ig.inertia().matrix();

  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  Ig_ = base_pose.rotation().transpose() * Ig_0 * base_pose.rotation();

  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
  auto rpy_m = toEulerAngles(base_pose.rotation());
  rpy_des_start =
      rpy_m - computeEulerAngleErr(rpy_m, rpy_traj->evaluate(time_cur));

  for (size_t k = 0; k <= N; k++) {
    if (k < N) {
      getDynamics(time_cur, k, mode_schedule);
    }
    getInequalityConstraints(k, N, mode_schedule);
    getCosts(time_cur, k, N, mode_schedule);
  }

  if (solution_.size() == N + 1) {
    solver_settings.warm_start = 1;
  } else {
    solver_settings.warm_start = 0;
    solution_.resize(N + 1);
  }
  hpipm::OcpQpIpmSolver solver(ocp_, solver_settings);

  vector_t x0(12);
  vector3_t omega0 = getJacobiFromRPYToOmega(rpy_des_start) *
                     rpy_traj->derivative(time_cur, 1);
  x0 << pos_traj->evaluate(time_cur), pos_traj->derivative(time_cur, 1),
      rpy_des_start, Ig_0 * omega0;

  const auto res = solver.solve(x0, ocp_, solution_);
  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    fitTraj(time_cur, N);
  } else {
    std::cout << "ConvexMPC: " << res << "\n";
    exit(0);
  }
  RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"),
              "ConvexMPC: generateTrajRef done");
}

void ConvexMPC::fitTraj(scalar_t time_cur, size_t N) {
  std::vector<scalar_t> time_array;
  std::vector<vector_t> base_pos_array;
  std::vector<vector_t> base_rpy_array;

  for (size_t k = 0; k < N; k++) {
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
  referenceBuffer_->setOptimizedBasePosTraj(base_pos_traj_ptr_);

  base_rpy_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline_hermite);
  base_rpy_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
  base_rpy_traj_ptr_->fit(time_array, base_rpy_array);
  referenceBuffer_->setOptimizedBaseRpyTraj(base_rpy_traj_ptr_);
}

vector3_t ConvexMPC::computeEulerAngleErr(const vector3_t &rpy_m,
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
