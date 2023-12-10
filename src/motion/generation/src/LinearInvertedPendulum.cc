#include "generation/LinearInvertedPendulum.h"
#include "generation/LegLogic.h"
#include <rcpputils/asserts.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

LinearInvertedPendulum::LinearInvertedPendulum(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
    std::shared_ptr<ReferenceBuffer> referenceReferenceBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      referenceBuffer_(referenceReferenceBuffer), nominal_z_(0.03) {

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

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  base_name = config_["model"]["base_name"].as<std::string>();
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
}

LinearInvertedPendulum::~LinearInvertedPendulum() {}

void LinearInvertedPendulum::get_dynamics(
    size_t k, const std::shared_ptr<ModeSchedule> mode_schedule) {

  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag = legged_robot::modeNumber2StanceLeg(
      mode_schedule->getModeFromPhase(phase));
  size_t nc = std::count(contact_flag.cbegin(), contact_flag.cend(), true);

  ocp_[k].A.setIdentity(4, 4);
  ocp_[k].A.block<2, 2>(0, 2).diagonal().fill(dt_);
  ocp_[k].B.setZero(4, 2);
  if (nc != 0) {
    ocp_[k].B.bottomRows(2).diagonal().fill(dt_);
  }
  ocp_[k].b.setZero(4);
}

void LinearInvertedPendulum::get_inequality_constraints(size_t k) {
  ocp_[k].idxbu = {0, 1};
  ocp_[k].lbu_mask.setOnes(2);
  ocp_[k].ubu_mask.setOnes(2);
  ocp_[k].lbu = -20.0 * vector_t::Ones(2);
  ocp_[k].ubu = 20.0 * vector_t::Ones(2);
}

void LinearInvertedPendulum::get_costs(
    scalar_t time_cur, size_t k, size_t N,
    const std::shared_ptr<ModeSchedule> mode_schedule) {
  auto footholds = referenceBuffer_->get_footholds();
  const scalar_t time_k = time_cur + k * dt_;
  const scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag = legged_robot::modeNumber2StanceLeg(
      mode_schedule->getModeFromPhase(phase));

  size_t nc = std::count(contact_flag.cbegin(), contact_flag.cend(), true);

  matrix_t C = matrix_t::Zero(2, 4);
  C.leftCols(2).setIdentity();
  const scalar_t lmd_inv = 0.5 / 9.81;
  matrix_t D = lmd_inv * matrix_t::Identity(2, 2);

  vector2_t y_des;
  if (nc == 1) {
    if (contact_flag[0]) {
      if (footholds[foot_names[0]].first < time_k) {
        y_des = footholds[foot_names[0]].second.head(2);
      } else {
        y_des = pinocchioInterface_ptr_->getFramePose(foot_names[0])
                    .translation()
                    .head(2);
      }
    } else if (contact_flag[1]) {
      if (footholds[foot_names[1]].first < time_k) {
        y_des = footholds[foot_names[1]].second.head(2);
      } else {
        y_des = pinocchioInterface_ptr_->getFramePose(foot_names[1])
                    .translation()
                    .head(2);
      }
    }
  } else {
    vector2_t p1, p2;
    if (footholds[foot_names[0]].first < time_k) {
      p1 = footholds[foot_names[0]].second.head(2);
    } else {
      p1 = pinocchioInterface_ptr_->getFramePose(foot_names[0])
               .translation()
               .head(2);
    }
    if (footholds[foot_names[1]].first < time_k) {
      p2 = footholds[foot_names[1]].second.head(2);
    } else {
      p2 = pinocchioInterface_ptr_->getFramePose(foot_names[1])
               .translation()
               .head(2);
    }
    y_des = 0.5 * (p1 + p2);
  }
  matrix_t Q = matrix_t::Zero(4, 4);
  matrix_t R = 1e-4 * matrix_t::Identity(2, 2);
  Q.diagonal() << 100, 100, 20, 20.0;

  ocp_[k].Q = C.transpose() * Q * C;
  ocp_[k].S = D.transpose() * Q * C;
  ocp_[k].q = -C.transpose() * Q * y_des;
  ocp_[k].R = D.transpose() * Q * D + R;
  ocp_[k].r = -D.transpose() * Q * y_des;

  if (k == N) {
    ocp_[k].Q.setZero();
    ocp_[k].S.setZero();
    ocp_[k].q.setZero();
    ocp_[k].R.setZero();
    ocp_[k].r.setZero();
  }
}

void LinearInvertedPendulum::optimize() {
  auto footholds = referenceBuffer_->get_footholds();
  const std::shared_ptr<ModeSchedule> mode_schedule =
      referenceBuffer_->get_mode_schedule();
  if (footholds.empty() && mode_schedule != nullptr) {
    return;
  }

  const scalar_t time_cur = nodeHandle_->now().seconds();

  size_t N = mode_schedule->duration() / dt_;
  ocp_.resize(N + 1);

  for (size_t k = 0; k <= N; k++) {
    if (k < N) {
      get_dynamics(k, mode_schedule);
    }
    get_inequality_constraints(k);
    get_costs(time_cur, k, N, mode_schedule);
  }

  if (solution_.size() == N + 1) {
    solver_settings.warm_start = 1;
  } else {
    solver_settings.warm_start = 0;
    solution_.resize(N + 1);
  }

  hpipm::OcpQpIpmSolver solver(ocp_, solver_settings);

  auto pos_m = pinocchioInterface_ptr_->getFramePose(base_name).translation();
  if (referenceBuffer_->get_base_pos_traj()) {
    auto pos_d = referenceBuffer_->get_base_pos_traj()->evaluate(time_cur);
    if ((pos_d - pos_m).norm() > 0.05) {
      pos_m = 0.05 * (pos_d - pos_m).normalized() + pos_m;
    }
  }

  vector_t x0(4);
  x0.head(4) << pos_m.head(2),
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name)
          .linear()
          .head(2);

  const auto res = solver.solve(x0, ocp_, solution_);

  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    has_sol_ = true;
    /* std::cout << "###########################################"
              << "\n";
    for (auto &sol : solution_) {
      std::cout << "forward: " << sol.x.transpose() << "\n";
    } */
    std::vector<scalar_t> time_array;
    std::vector<vector_t> base_pos_array;

    for (size_t k = 1; k <= N; k++) {
      vector3_t pos;
      pos << solution_[k].x.head(2), 0.5;
      time_array.emplace_back(time_cur + k * dt_);
      base_pos_array.emplace_back(pos);
    }
    auto base_pos_ref_ptr_ = std::make_shared<CubicSplineTrajectory>(
        3, CubicSplineInterpolation::SplineType::cspline);
    vector3_t acc_s, vel_e;
    acc_s << solution_[0].u.head(2), 0.0;
    vel_e << solution_[N].x.segment(2, 2), 0.0;
    base_pos_ref_ptr_->set_boundary(
        CubicSplineInterpolation::BoundaryType::second_deriv, acc_s,
        CubicSplineInterpolation::BoundaryType::first_deriv, vel_e);
    base_pos_ref_ptr_->fit(time_array, base_pos_array);
    referenceBuffer_->set_base_pos_traj(base_pos_ref_ptr_);
  }
}

} // namespace clear
