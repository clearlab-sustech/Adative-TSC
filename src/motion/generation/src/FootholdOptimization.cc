#include "generation/FootholdOptimization.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

FootholdOptimization::FootholdOptimization(
    Node::SharedPtr nodeHandle,
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
  ocp_[k].lbu = -5.0 * dt * vector_t::Ones(3 * nf);
  ocp_[k].ubu = 5.0 * dt * vector_t::Ones(3 * nf);
}

void FootholdOptimization::add_costs(scalar_t t, size_t k) {
  const scalar_t tk = t + k * dt;
  auto base_pos_ref_traj = referenceBuffer_->getOptimizedBasePosTraj();
  auto base_rpy_traj = referenceBuffer_->getOptimizedBaseRpyTraj();

  if (base_pos_ref_traj.get() == nullptr) {
    base_pos_ref_traj = referenceBuffer_->getIntegratedBasePosTraj();
  }
  if (base_rpy_traj.get() == nullptr) {
    base_rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  }

  vector3_t pos_des = base_pos_ref_traj->evaluate(tk);
  vector3_t rpy_des = base_rpy_traj->evaluate(tk);

  vector_t x_des(3 * nf);
  vector3_t v_des = toRotationMatrix(rpy_des).transpose() *
                    base_pos_ref_traj->derivative(tk, 1);
  for (size_t i = 0; i < nf; i++) {
    vector3_t pn = footholds_nominal_pos[foot_names[i]];
    pn.y() *= (abs(v_des.y()) > 0.05 ? 1.0 : 1.0);
    x_des.segment(3 * i, 3) =
        pos_des +
        toRotationMatrix(rpy_des) * pn;
    x_des(3 * i + 2) = 0.02;
  }

  // std::cout << "xdes: " << x_des.transpose() << "\n";

  matrix_t Q = 1e2 * matrix_t::Identity(3 * nf, 3 * nf);
  ocp_[k].Q = Q;
  ocp_[k].q = -Q * x_des;
  ocp_[k].S.setZero(3 * nf, 3 * nf);
  ocp_[k].R = 1e-3 * matrix_t::Identity(3 * nf, 3 * nf);
  ocp_[k].r.setZero(3 * nf);
}

void FootholdOptimization::optimize() {
  footholds_ = std::map<std::string, std::pair<scalar_t, vector3_t>>();
  auto base_pos_ref_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto base_rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto mode_schedule = referenceBuffer_->getModeSchedule();
  if (mode_schedule.get() == nullptr || base_pos_ref_traj.get() == nullptr ||
      base_rpy_traj.get() == nullptr) {
    referenceBuffer_->setFootholds(footholds_);
  }
  const scalar_t t = nodeHandle_->now().seconds();
  const size_t N = static_cast<size_t>(mode_schedule->duration() / dt);
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
  heuristic1();
  referenceBuffer_->setFootholds(footholds_);
}

void FootholdOptimization::heuristic1() {

  const scalar_t t = nodeHandle_->now().seconds();

  auto base_pos_ref_traj = referenceBuffer_->getOptimizedBasePosTraj();
  auto base_rpy_traj = referenceBuffer_->getOptimizedBaseRpyTraj();
  auto mode_schedule = referenceBuffer_->getModeSchedule();

  vector3_t v_des = base_pos_ref_traj->derivative(t, 1);

  const auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  const auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
  /* auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(0.0));
   */

  const scalar_t p_rel_max = 0.5f;
  const scalar_t p_rel_y_max = 0.4f;

  vector3_t rpy_dot =
      getJacobiFromOmegaToRPY(toEulerAngles(base_pose.rotation())) *
      base_twist.angular();

  for (size_t i = 0; i < nf; i++) {
    const auto &foot_name = foot_names[i];
    scalar_t pfx_rel, pfy_rel;
    scalar_t x_fb = abs(base_twist.linear().x() - v_des.x()) > 0.1
                        ? base_twist.linear().x() - v_des.x()
                        : 0.0;
    scalar_t y_fb = abs(base_twist.linear().y() - v_des.y()) > 0.1
                        ? base_twist.linear().y() - v_des.y()
                        : 0.0;
    pfx_rel = 0.1 * x_fb;
    pfy_rel = 0.1 * y_fb;
    pfx_rel = std::min(std::max(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = std::min(std::max(pfy_rel, -p_rel_y_max), p_rel_y_max);
    footholds_[foot_name].second.x() += pfx_rel;
    footholds_[foot_name].second.y() += pfy_rel;
    footholds_[foot_name].second.z() = 0.02;
  }
}

void FootholdOptimization::heuristic2() {

  const scalar_t t = nodeHandle_->now().seconds();

  auto base_pos_ref_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto base_rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto mode_schedule = referenceBuffer_->getModeSchedule();

  vector3_t v_des = base_pos_ref_traj->derivative(t, 1);

  const auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  const auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  vector3_t rpy_dot =
      getJacobiFromOmegaToRPY(toEulerAngles(base_pose.rotation())) *
      base_twist.angular();

  auto swtr = quadruped::getTimeOfNextTouchDown(0, mode_schedule);

  const scalar_t p_rel_x_max = 0.5f;
  const scalar_t p_rel_y_max = 0.4f;

  for (size_t i = 0; i < nf; i++) {
    const auto &foot_name = foot_names[i];
    const scalar_t nextStanceTime = swtr[i];

    vector3_t pYawCorrected =
        toRotationMatrix(base_rpy_traj->evaluate(t + nextStanceTime)) *
        footholds_nominal_pos[foot_name];

    scalar_t pfx_rel, pfy_rel;
    std::pair<scalar_t, vector3_t> foothold;
    foothold.first = nextStanceTime + t;
<<<<<<< HEAD
    foothold.second = base_pose.translation() +
                      (pYawCorrected + std::max(0.0, nextStanceTime) * v_des);
=======
    foothold.second =
        base_pose.translation() +
        (pYawCorrected + std::max(0.0, nextStanceTime) * v_des);
>>>>>>> bbd32c6f4d1ad268f36378b3bb3a975da76236b3
    pfx_rel = 0.5 * mode_schedule->duration() * v_des.x() +
              0.01 * (base_twist.linear().x() - v_des.x()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (base_twist.linear().y() * rpy_dot.z());
    pfy_rel = 0.5 * mode_schedule->duration() * v_des.y() +
              0.01 * (base_twist.linear().y() - v_des.y()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (-base_twist.linear().x() * rpy_dot.z());
    pfx_rel = std::min(std::max(pfx_rel, -p_rel_x_max), p_rel_x_max);
    pfy_rel = std::min(std::max(pfy_rel, -p_rel_y_max), p_rel_y_max);
    foothold.second.x() += pfx_rel;
    foothold.second.y() += pfy_rel;
    foothold.second.z() = 0.023;
    footholds_[foot_name] = foothold;
  }
}

} // namespace clear