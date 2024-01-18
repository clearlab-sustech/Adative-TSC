#include "generation/LipGen.h"
#include "generation/LegLogic.h"
#include <rcpputils/asserts.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

LipGen::LipGen(Node::SharedPtr nodeHandle,
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
  scalar_t freq_ = config_["generation"]["frequency"].as<scalar_t>();
  dt_ = 1.0 / freq_;

  vector_t qpos, qvel;
  qpos.setZero(pinocchioInterface_ptr_->nq());
  qvel.setZero(pinocchioInterface_ptr_->nv());
  pinocchioInterface_ptr_->updateRobotState(qpos, qvel);
  for (const auto &foot : foot_names) {
    footholds_nominal_pos[foot] =
        pinocchioInterface_ptr_->getFramePose(foot).translation();
    footholds_nominal_pos[foot].z() = 0.0;
    footholds_nominal_pos[foot].y() *= 0.8;
    footholds_nominal_pos[foot].x() = pinocchioInterface_ptr_->getCoMPos().x();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("FootholdOptimization"),
                       foot << " nominal pos: "
                            << footholds_nominal_pos[foot].transpose());
  }

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
  t0 = nodeHandle->now().seconds();
  RCLCPP_INFO(rclcpp::get_logger("LipGen"), "LipGen: Construction done");
}

LipGen::~LipGen() {}

void LipGen::optimize() {
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto pos_ref = referenceBuffer_->getIntegratedBasePosTraj();
  if (pos_ref == nullptr || rpy_traj == nullptr) {
    return;
  }

  const scalar_t time_cur = nodeHandle_->now().seconds();

  size_t N = pos_ref->duration() / dt_;
  ocp_.resize(N + 1);

  const std::shared_ptr<ModeSchedule> mode_schedule =
      referenceBuffer_->getModeSchedule();
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

  auto pos_m = pinocchioInterface_ptr_->getFramePose(base_name).translation();
  vector3_t pos_d = pos_ref->evaluate(time_cur);
  if ((pos_d - pos_m).norm() > 0.05) {
    pos_m = 0.05 * (pos_d - pos_m).normalized() + pos_m;
  }

  size_t nf = foot_names.size();
  vector_t x0(4 + 2 * nf);
  x0.head(4) << pos_m.head(2),
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name)
          .linear()
          .head(2);
  for (size_t i = 0; i < nf; i++) {
    auto foot_pose = pinocchioInterface_ptr_->getFramePose(foot_names[i]);
    x0.segment(4 + 2 * i, 2) = foot_pose.translation().head(2);
  }

  const auto res = solver.solve(x0, ocp_, solution_);
  auto footholds_ = std::map<std::string, std::pair<scalar_t, vector3_t>>();

  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    std::cout << "###########################################"
              << "\n";
    for (auto &sol : solution_) {
      std::cout << "forward: " << sol.x.transpose() << "\n";
    }
    fitTraj(time_cur, N);
  }
}

void LipGen::setVelCmd(vector3_t vel) { vel_cmd = vel; }

void LipGen::setHeightCmd(scalar_t h) { h_des = h; }

void LipGen::generateTrajRef() {
  const scalar_t t_now = nodeHandle_->now().seconds();
  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);

  scalar_t magn = 0;
  scalar_t alpha = 0.75;
  if (t_now - t0 > 6.0) {
    magn = 0.0 * (sin(alpha * (t_now - t0)) > 0 ? 1.0 : -1.0);
  }

  if (first_run) {
    first_run = false;
    pos_start = base_pose_m.translation();
    pos_start.z() = h_des;
  } else {
    if ((base_pose_m.translation() - pos_start).norm() < 0.2) {
      pos_start += dt_ * vector3_t(magn, 0, 0.0);
      pos_start.z() = h_des;
    }
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> pos_t;
  scalar_t horizon_time = referenceBuffer_->getModeSchedule()->duration();
  size_t N = horizon_time / 0.05;
  for (size_t k = 0; k < N; k++) {
    time.push_back(t_now + 0.05 * k);
    // vector3_t vel_des = toRotationMatrix(rpy_k) * vel_cmd;
    vector3_t vel_des = vector3_t(magn, 0, 0.0);
    vector3_t pos_k = pos_start + 0.05 * k * vel_des;
    pos_k.z() = h_des;
    pos_t.emplace_back(pos_k);
  }

  auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_pos->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector3_t(magn, 0, 0.0),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_pos->fit(time, pos_t);
  referenceBuffer_->setIntegratedBasePosTraj(cubicspline_pos);
}

void LipGen::getDynamics(scalar_t time_cur, size_t k,
                         const std::shared_ptr<ModeSchedule> mode_schedule) {
  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  size_t nc = std::count(contact_flag.cbegin(), contact_flag.cend(), true);

  const scalar_t lmd = 9.81 / h_des;
  ocp_[k].A.setIdentity(8, 8);
  ocp_[k].A.block<2, 2>(0, 2).diagonal().fill(dt_);
  ocp_[k].A.block<2, 2>(2, 0).diagonal().fill(lmd * dt_);

  ocp_[k].B.setZero(8, 4);
  ocp_[k].B.bottomRows(4).diagonal().fill(dt_);
  if (nc = 1) {
    if (contact_flag[0]) {
      ocp_[k].A.block<2, 2>(2, 4).diagonal().fill(-lmd * dt_);
      ocp_[k].B.block<2, 2>(4, 0).setZero();
    } else {
      ocp_[k].A.block<2, 2>(2, 6).diagonal().fill(-lmd * dt_);
      ocp_[k].B.block<2, 2>(6, 2).setZero();
    }
  } else if (nc == 2) {
    ocp_[k].A.block<2, 2>(2, 4).diagonal().fill(-0.5 * lmd * dt_);
    ocp_[k].A.block<2, 2>(2, 6).diagonal().fill(-0.5 * lmd * dt_);
    ocp_[k].B.setZero();
  }
  ocp_[k].b.setZero(8);
}

void LipGen::getInequalityConstraints(
    size_t k, size_t N, const std::shared_ptr<ModeSchedule> mode_schedule) {
  if (k < N) {
    ocp_[k].idxbu.clear();
    for (size_t i = 0; i < 2; i++) {
      ocp_[k].idxbu.emplace_back(i * 2);
      ocp_[k].idxbu.emplace_back(i * 2 + 1);
    }
    ocp_[k].lbu_mask.setOnes(4);
    ocp_[k].ubu_mask.setOnes(4);
    ocp_[k].lbu = -10.0 * vector_t::Ones(4);
    ocp_[k].ubu = 10.0 * vector_t::Ones(4);
  }
}

void LipGen::getCosts(scalar_t time_cur, size_t k, size_t N,
                      const std::shared_ptr<ModeSchedule> mode_schedule) {
  const scalar_t time_k = time_cur + k * dt_;
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto pos_ref = referenceBuffer_->getIntegratedBasePosTraj();

  vector3_t base_pos_des = pos_ref->evaluate(time_k);
  vector3_t base_vel_des = pos_ref->derivative(time_k, 1);
  vector3_t base_rpy_des = rpy_traj->evaluate(time_k);
  matrix3_t wRb = toRotationMatrix(base_rpy_des);

  vector_t x_des = vector_t::Zero(8);
  x_des.head(4) << base_pos_des.head(2), base_vel_des.head(2);
  weight_.setZero(8, 8);
  weight_.diagonal() << 100, 100, 20, 20.0, 500, 5000, 500, 5000;
  for (size_t i = 0; i < 2; i++) {
    vector3_t shift = wRb * footholds_nominal_pos[foot_names[i]];
    x_des.segment(4 + 2 * i, 2) = shift.head(2) + base_pos_des.head(2);
  }
  // std::cout << "x_des: " << x_des.transpose() << "\n";

  ocp_[k].Q = weight_;
  ocp_[k].S = matrix_t::Zero(4, 8);
  ocp_[k].q = -weight_ * x_des;
  ocp_[k].R = 1e-6 * matrix_t::Identity(4, 4);
  ocp_[k].r.setZero(4);

  if (k == N) {
    ocp_[k].Q = 1e3 * ocp_[k].Q;
    ocp_[k].q = 1e3 * ocp_[k].q;
  }
}

void LipGen::fitTraj(scalar_t time_cur, size_t N) {
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto pos_ref = referenceBuffer_->getIntegratedBasePosTraj();

  std::vector<scalar_t> time_array;
  std::vector<vector_t> base_pos_array;
  std::vector<vector_t> base_vel_array;

  for (size_t k = 1; k <= N; k++) {
    vector3_t pos, vel;
    pos << solution_[k].x.head(2), h_des;
    vel << solution_[k].x.segment(2, 2), 0.0;
    time_array.emplace_back(time_cur + k * dt_);
    base_pos_array.emplace_back(pos);
    base_vel_array.emplace_back(vel);
  }
  auto base_pos_ref_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  vector3_t acc_s, vel_e;
  acc_s << solution_[0].u.head(2), 0.0;
  vel_e << solution_[N].x.segment(2, 2),
      pos_ref->derivative(time_cur + N * dt_, 1).z();
  base_pos_ref_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_vel_array.front(),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_vel_array.back());
  base_pos_ref_ptr_->fit(time_array, base_pos_array);
  referenceBuffer_->setOptimizedBasePosTraj(base_pos_ref_ptr_);

  auto base_vel_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  vector3_t acc_s, acc_e;
  vector_t xdot = 1.0 / dt_ *
                  (ocp_[0].A * solution_[0].x + ocp_[0].B * solution_[0].u -
                   solution_[0].x);
  acc_s << xdot.segment(2, 2), 0.0;
  xdot = 1.0 / dt_ *
                  (ocp_[0].A * solution_[0].x + ocp_[0].B * solution_[0].u -
                   solution_[0].x);
  acc_s << xdot.segment(2, 2), 0.0;
  base_vel_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      acc_s.front(),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_vel_array.back());
  base_vel_traj_ptr_->fit(time_array, base_pos_array);
  referenceBuffer_->setOptimizedBaseVelTraj(base_vel_traj_ptr_);

  auto footholds_ = std::map<std::string, std::pair<scalar_t, vector3_t>>();
  auto swtr =
      biped::getTimeOfNextTouchDown(0.0, referenceBuffer_->getModeSchedule());
  for (size_t i = 0; i < foot_names.size(); i++) {
    size_t idx = std::min(static_cast<size_t>(swtr[i] / dt_) + 1, N);
    footholds_[foot_names[i]].first = time_cur + idx * dt_;
    footholds_[foot_names[i]].second.head(2) =
        solution_[idx].x.segment(4 + 2 * i, 2);
    footholds_[foot_names[i]].second.z() = 0.02;
  }
  referenceBuffer_->setFootholds(footholds_);
}

} // namespace clear
