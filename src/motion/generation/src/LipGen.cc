#include "generation/LipGen.h"
#include <core/gait/LegLogic.h>
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
    footholds_nominal_pos[foot].x() =
        pinocchioInterface_ptr_->getCoMPos().x() - 0.02;
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

  const std::shared_ptr<ModeSchedule> mode_schedule =
      referenceBuffer_->getModeSchedule();
  if (mode_schedule == nullptr) {
    return;
  }
  generateTrajRef();
  const scalar_t time_cur = nodeHandle_->now().seconds();
  size_t N = mode_schedule->duration() / dt_;
  ocp_.resize(N + 1);
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
  // vector3_t pos_d = pos_ref_ptr_->evaluate(time_cur);
  // if ((pos_d - pos_m).norm() > 0.02) {
  //   pos_m = 0.02 * (pos_d - pos_m).normalized() + pos_m;
  // }

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
    // std::cout << "###########################################"
    //           << "\n";
    // for (auto &sol : solution_) {
    //   std::cout << "forward: " << sol.x.transpose() << "\n";
    // }
    fitTraj(time_cur, N);
  }
}

void LipGen::setVelCmd(vector3_t vd, scalar_t yawd) {
  vel_cmd = vd;
  yawd_ = yawd;
}

void LipGen::setHeightCmd(scalar_t h) { h_des = h; }

void LipGen::generateTrajRef() {
  const scalar_t t_now = nodeHandle_->now().seconds();
  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);

  if (first_run) {
    first_run = false;
    pos_start = base_pose_m.translation();
    pos_start.z() = h_des;
  } else {
    vector3_t vw = base_pose_m.rotation() * vel_cmd;
    if ((base_pose_m.translation() - pos_start).norm() < 0.2) {
      pos_start += dt_ * vw;
      pos_start.z() = h_des;
    }
    // pos_start.head(2) = base_pose_m.translation().head(2) + dt_ * vw.head(2);
    pos_start.z() = h_des;
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> rpy_t, pos_t;
  scalar_t horizon_time = referenceBuffer_->getModeSchedule()->duration();
  size_t N = horizon_time / 0.05;
  vector_t rpy_c = toEulerAngles(base_pose_m.rotation());
  for (size_t k = 0; k < N; k++) {
    time.push_back(t_now + 0.05 * k);
    vector3_t rpy_k = rpy_c;
    rpy_k.z() += 0.05 * k * yawd_;

    vector3_t vel_des = toRotationMatrix(rpy_k) * vel_cmd;
    vector3_t pos_k = pos_start + 0.05 * k * vel_des;
    pos_k.z() = h_des;
    pos_t.emplace_back(pos_k);
  }

  pos_ref_ptr_ = std::make_shared<CubicSplineTrajectory>(3);
  pos_ref_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_pose_m.rotation() * vel_cmd,
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  pos_ref_ptr_->fit(time, pos_t);
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

    ocp_[k].C = matrix_t::Zero(2, 8);
    ocp_[k].C.leftCols(2).setIdentity();
    ocp_[k].C.rightCols(2).diagonal().fill(-1);
    ocp_[k].D.setZero(2, 4);
    ocp_[k].lg = -0.3 * vector2_t::Ones();
    ocp_[k].ug = 0.3 * vector2_t::Ones();
    ocp_[k].lg_mask.setOnes(2);
    ocp_[k].ug_mask.setOnes(2);
  }
}

void LipGen::getCosts(scalar_t time_cur, size_t k, size_t N,
                      const std::shared_ptr<ModeSchedule> mode_schedule) {
  const scalar_t time_k = time_cur + k * dt_;
  vector3_t base_pos_des = pos_ref_ptr_->evaluate(time_k);
  vector3_t base_vel_des = pos_ref_ptr_->derivative(time_k, 1);

  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);
  vector3_t rpy_k = toEulerAngles(base_pose_m.rotation()) +
                    0.05 * k * yawd_ * vector3_t::UnitZ();
  matrix3_t wRb = toRotationMatrix(rpy_k);

  vector_t x_des = vector_t::Zero(8);
  x_des.head(4) << base_pos_des.head(2), base_vel_des.head(2);
  weight_.setZero(8, 8);
  weight_.diagonal() << 20, 4, 4.0, 0.1, 500, 500, 500, 500;
  weight_.topLeftCorner<2, 2>() = wRb.topLeftCorner<2, 2>() *
                                  weight_.topLeftCorner<2, 2>() *
                                  wRb.topLeftCorner<2, 2>().transpose();
  weight_.block<2, 2>(2, 2) = wRb.topLeftCorner<2, 2>() *
                              weight_.block<2, 2>(2, 2) *
                              wRb.topLeftCorner<2, 2>().transpose();
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
  std::vector<scalar_t> time_array;
  std::vector<vector_t> base_pos_array;
  std::vector<vector_t> base_vel_array;
  std::vector<vector_t> foot_pos_array1;
  std::vector<vector_t> foot_pos_array2;

  for (size_t k = 1; k <= N; k++) {
    vector3_t pos, vel;
    pos << solution_[k].x.head(2), h_des;
    vel << solution_[k].x.segment(2, 2), 0.0;
    time_array.emplace_back(time_cur + k * dt_);
    base_pos_array.emplace_back(pos);
    base_vel_array.emplace_back(vel);

    vector3_t fpos1, fpos2;
    fpos1 << solution_[k].x.segment(4, 2), 0.02;
    fpos2 << solution_[k].x.tail(2), 0.02;
    foot_pos_array1.emplace_back(fpos1);
    foot_pos_array2.emplace_back(fpos2);
  }

  auto base_pos_ref_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_pos_ref_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_vel_array.front(),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_vel_array.back());
  base_pos_ref_ptr_->fit(time_array, base_pos_array);
  referenceBuffer_->setLipBasePosTraj(base_pos_ref_ptr_);

  auto base_vel_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_vel_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[1].x.segment(2, 2) - solution_[0].x.segment(2, 2)),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ *
          (solution_[N - 1].x.segment(2, 2) -
           solution_[N - 2].x.segment(2, 2)));
  base_vel_traj_ptr_->fit(time_array, base_vel_array);
  referenceBuffer_->setLipBaseVelTraj(base_vel_traj_ptr_);

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

  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>> foot_pos_traj;
  auto cubicspline_fpos1 = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  cubicspline_fpos1->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
  cubicspline_fpos1->fit(time_array, foot_pos_array1);
  foot_pos_traj[foot_names[0]] = std::move(cubicspline_fpos1);

  auto cubicspline_fpos2 = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  cubicspline_fpos2->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
  cubicspline_fpos2->fit(time_array, foot_pos_array2);
  foot_pos_traj[foot_names[1]] = std::move(cubicspline_fpos2);
  referenceBuffer_->setLipFootPosTraj(foot_pos_traj);
}

} // namespace clear
