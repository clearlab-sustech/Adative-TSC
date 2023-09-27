#include "generation/FootholdOptimization.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

FootholdOptimization::FootholdOptimization(
    std::string config_yaml,
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
    std::shared_ptr<TrajectoriesArray> referenceTrajectoriesBuffer)
    : pinocchioInterface_ptr_(pinocchioInterface_ptr),
      referenceTrajectoriesBuffer_(referenceTrajectoriesBuffer) {

  auto config_ = YAML::LoadFile(config_yaml);

  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  base_name = config_["model"]["base_name"].as<std::string>();

  vector_t qpos, qvel;
  qpos.setZero(pinocchioInterface_ptr_->nq());
  qvel.setZero(pinocchioInterface_ptr_->nv());
  pinocchioInterface_ptr_->updateRobotState(qpos, qvel);
  for (const auto &foot : foot_names) {
    footholds_nominal_pos[foot] =
        pinocchioInterface_ptr_->getFramePose(foot).translation();
    footholds_nominal_pos[foot].z() = 0.0;
    footholds_nominal_pos[foot].y() *= 0.8;
  }

  nf = foot_names.size();
  dt = 0.05;

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
  solver_settings.warm_start = 0;

  amazeModel_ptr_ = std::make_shared<AmazeModel>(*pinocchioInterface_ptr_,
                                                 referenceTrajectoriesBuffer_);
}

FootholdOptimization::~FootholdOptimization() {}

void FootholdOptimization::add_dynamics(
    size_t k, const std::shared_ptr<ModeSchedule> mode_schedule) {
  ocp_[k].A.setIdentity(3 * nf, 3 * nf);
  ocp_[k].B.setZero(3 * nf, 3 * nf);
  ocp_[k].b.setZero(3 * nf);

  scalar_t phase = k * dt / mode_schedule->duration();
  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  for (size_t i = 0; i < nf; i++) {
    if (!contact_flag[i]) {
      ocp_[k].B.block<3, 3>(3 * i, 3 * i).setIdentity();
    }
  }
}

void FootholdOptimization::add_constraints(size_t k) {
  ocp_[k].idxbu.clear();
  for (size_t i = 0; i < 3 * nf; i++) {
    ocp_[k].idxbu.emplace_back(i);
  }
  ocp_[k].lbu_mask.setOnes(3 * nf);
  ocp_[k].ubu_mask.setOnes(3 * nf);
  ocp_[k].lbu = -2.0 * dt * vector_t::Ones(3 * nf);
  ocp_[k].ubu = 2.0 * dt * vector_t::Ones(3 * nf);
}

void FootholdOptimization::add_costs(scalar_t t, size_t k) {
  const scalar_t tk = t + k * dt;
  auto base_pos_ref_traj = amazeModel_ptr_->get_base_pos_trajectory();
  auto base_rpy_traj = amazeModel_ptr_->get_base_rpy_trajectory();

  if (base_pos_ref_traj.get() == nullptr) {
    base_pos_ref_traj =
        referenceTrajectoriesBuffer_.get()->get_base_pos_ref_traj();
  }
  if (base_rpy_traj.get() == nullptr) {
    base_rpy_traj = referenceTrajectoriesBuffer_.get()->get_base_rpy_traj();
  }

  vector3_t pos_des = base_pos_ref_traj->evaluate(tk);
  vector3_t rpy_des = base_rpy_traj->evaluate(tk);
  vector3_t vel_des = toRotationMatrix(rpy_des).transpose() *
                      base_pos_ref_traj->derivative(tk, 1);

  vector_t x_des(3 * nf);
  for (size_t i = 0; i < nf; i++) {
    vector3_t expansion_y =
        (i % 2 == 0 ? 1.0 : -1.0) *
        (std::min(0.2, 0.2 * (1.0 - exp(-abs(vel_des.y())))) +
         0.05 * (abs(vel_des.x()) > 0.2 ? -0.4 : 0.0)) *
        vector3_t::UnitY();
    x_des.segment(3 * i, 3) =
        pos_des + toRotationMatrix(rpy_des) *
                      (footholds_nominal_pos[foot_names[i]] + expansion_y);
    x_des(3 * i + 2) = 0.02;
  }

  matrix_t Q = 1e2 * matrix_t::Identity(3 * nf, 3 * nf);
  ocp_[k].Q = Q;
  ocp_[k].q = -Q * x_des;
  ocp_[k].S.setZero(3 * nf, 3 * nf);
  ocp_[k].R = 1e-2 * matrix_t::Identity(3 * nf, 3 * nf);
  ocp_[k].r.setZero(3 * nf);
}

std::map<std::string, std::pair<scalar_t, vector3_t>>
FootholdOptimization::optimize(
    scalar_t t, const std::shared_ptr<ModeSchedule> mode_schedule) {
  auto base_pos_ref_traj =
      referenceTrajectoriesBuffer_.get()->get_base_pos_ref_traj();
  auto base_rpy_traj = referenceTrajectoriesBuffer_.get()->get_base_rpy_traj();
  if (mode_schedule.get() == nullptr || base_pos_ref_traj.get() == nullptr ||
      base_rpy_traj.get() == nullptr) {
    return footholds_;
  }
  amazeModel_ptr_->optimize(t, mode_schedule);

  size_t N = static_cast<size_t>(mode_schedule->duration() / dt);
  ocp_.clear();
  ocp_.resize(N + 1);

  if (solution_.size() == N + 1) {
    solver_settings.warm_start = 1;
  } else {
    solver_settings.warm_start = 0;
    solution_.resize(N + 1);
  }

  for (size_t k = 0; k <= N; k++) {
    add_dynamics(k, mode_schedule);
    add_constraints(k);
    add_costs(t, k);
  }

  vector_t x0(3 * nf);
  for (size_t k = 0; k < nf; k++) {
    x0.segment(3 * k, 3) =
        pinocchioInterface_ptr_->getFramePose(foot_names[k]).translation();
  }

  footholds_ = std::map<std::string, std::pair<scalar_t, vector3_t>>();
  hpipm::OcpQpIpmSolver solver(ocp_, solver_settings);
  const auto res = solver.solve(x0, ocp_, solution_);
  auto swtr = quadruped::getTimeOfNextTouchDown(0, mode_schedule);
  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    /* std::cout << "###########################################"
              << "\n";
    for (auto &sol : solution_) {
      std::cout << "forward: " << sol.x.transpose() << "\n";
    } */
    for (size_t k = 0; k < nf; k++) {
      std::pair<scalar_t, vector3_t> foothold;
      size_t idx =
          std::min(static_cast<size_t>(swtr[k] / dt) + 1, solution_.size() - 1);
      foothold.first = swtr[k] + t;
      foothold.second = solution_[idx].x.segment(3 * k, 3);
      footholds_[foot_names[k]] = std::move(foothold);
    }
  } else {
    std::cout << "FootholdOptimization: " << res << "\n";
  }
  return footholds_;
}

} // namespace clear