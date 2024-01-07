#include "control/ConstructVectorField.h"
#include <pinocchio/Orientation.h>
#include <rcpputils/asserts.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

ConstructVectorField::ConstructVectorField(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr)
    : nodeHandle_(nodeHandle), pinocchioInterfacePtr_(pinocchioInterfacePtr) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);

  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();

  base_name = config_["model"]["base_name"].as<std::string>();

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

ConstructVectorField::~ConstructVectorField() {}

void ConstructVectorField::updateReferenceBuffer(
    std::shared_ptr<ReferenceBuffer> referenceBuffer) {
  referenceBuffer_ = referenceBuffer;
}

void ConstructVectorField::add_linear_system(size_t k) {
  const scalar_t time_k = time_now_ + k * dt_;
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto foot_traj = referenceBuffer_->getFootPosTraj();
  auto mode_schedule = referenceBuffer_->getModeSchedule();

  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  const size_t nf = foot_names.size();
  rcpputils::assert_true(nf == contact_flag.size());

  auto base_pose = pinocchioInterfacePtr_->getFramePose(base_name);
  vector3_t rpy = toEulerAngles(base_pose.rotation());

  ocp_[k].A.setIdentity(12, 12);
  ocp_[k].A.block<3, 3>(0, 3).diagonal().fill(dt_);
  ocp_[k].A.block<3, 3>(6, 9) = dt_ * getJacobiFromOmegaToRPY(rpy);
  ocp_[k].B.setZero(12, nf * 3);
  vector3_t xc = phase * pos_traj->evaluate(time_k) + (1.0 - phase) * base_pose.translation();
  for (size_t i = 0; i < nf; i++) {
    const auto &foot_name = foot_names[i];
    if (contact_flag[i]) {
      vector3_t pf_i = foot_traj[foot_name]->evaluate(time_k);
      ocp_[k].B.middleRows(3, 3).middleCols(3 * i, 3).diagonal().fill(
          dt_ / total_mass_);
      ocp_[k].B.bottomRows(3).middleCols(3 * i, 3) =
          Ig_.inverse() * skew(dt_ * (pf_i - xc));
    }
  }

  ocp_[k].b.setZero(12);
  ocp_[k].b(5) += -dt_ * grav_;
  
}

void ConstructVectorField::add_state_input_constraints(size_t k, size_t N) {
  const size_t nf = foot_names.size();
  auto mode_schedule = referenceBuffer_->getModeSchedule();

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

void ConstructVectorField::add_cost(size_t k, size_t N) {
  const size_t nf = foot_names.size();
  const scalar_t time_k = time_now_ + k * dt_;
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto mode_schedule = referenceBuffer_->getModeSchedule();

  vector3_t rpy_des = rpy_traj->evaluate(time_k) -
                      rpy_traj->evaluate(time_now_) + rpy_des_start;
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
    ocp_[k].R = 2e-4 * matrix_t::Identity(3 * nf, 3 * nf);
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
    // ocp_[k].Q = 1e2 * ocp_[k].Q;
    // ocp_[k].q = 1e2 * ocp_[k].q;
  }
}

std::shared_ptr<ConstructVectorField::VectorFieldParam>
ConstructVectorField::compute() {
  feedback_law_ptr = nullptr;

  if (!referenceBuffer_->isReady()) {
    return feedback_law_ptr;
  }
  
  time_now_ = nodeHandle_->now().seconds();

  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  
  size_t N = rpy_traj->duration() / dt_;
  ocp_.resize(N + 1);

  Ig_ = pinocchioInterfacePtr_->getData().Ig.inertia().matrix();

  auto base_pose = pinocchioInterfacePtr_->getFramePose(base_name);
  auto base_twist =
      pinocchioInterfacePtr_->getFrame6dVel_localWorldAligned(base_name);
  auto rpy_m = toEulerAngles(base_pose.rotation());
  rpy_des_start =
      rpy_m - computeEulerAngleErr(rpy_m, rpy_traj->evaluate(time_now_));

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
  x0 << base_pose.translation(), base_twist.linear(), rpy_m,
      base_twist.angular();

  const auto res = solver.solve(x0, ocp_, solution_);

  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    has_sol_ = true;
    feedback_law_ptr = std::make_shared<VectorFieldParam>();
    matrix_t P(6, 12);
    P.setZero();
    P.topRows(3).middleCols(3, 3).setIdentity();
    P.bottomRows(3).middleCols(9, 3).setIdentity();

    matrix_t A = 1.0 / dt_ * (ocp_[0].A - matrix_t::Identity(12, 12));
    matrix_t B = 1.0 / dt_ * ocp_[0].B;
    vector_t drift = 1.0 / dt_ * ocp_[0].b;
    feedback_law_ptr->K = P * (A + B * solution_[0].K);
    feedback_law_ptr->b = P * (B * solution_[0].k + drift);
    feedback_law_ptr->force_des = solution_[0].u;

    std::vector<scalar_t> time_array;
    std::vector<vector_t> base_pos_array;
    std::vector<vector_t> base_rpy_array;

    for (size_t k = 0; k <= N; k++) {
      time_array.emplace_back(time_now_ + k * dt_);
      base_pos_array.emplace_back(solution_[k].x.head(3));
      base_rpy_array.emplace_back(solution_[k].x.segment(6, 3));
    }
    /* std::cout << "#####################acc opt1######################\n"
              << (A * x0 + B * solution_[0].u).transpose()
              << "\n"; */
    // std::cout << "###########################################"
    //           << "\n";
    // for (auto &sol : solution_) {
    //   std::cout << "forward: " << sol.x.transpose() << "\n";
    // }
  } else {
    std::cout << "ConstructVectorField: " << res << "\n";
  }
  return feedback_law_ptr;
}

} // namespace clear
