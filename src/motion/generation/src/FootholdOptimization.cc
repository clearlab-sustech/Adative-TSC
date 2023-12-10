#include "generation/FootholdOptimization.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

FootholdOptimization::FootholdOptimization(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
    std::shared_ptr<ReferenceBuffer> referenceTrajectoriesBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      refTrajBuffer_(referenceTrajectoriesBuffer) {

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
    footholds_nominal_pos[foot].x() = pinocchioInterface_ptr_->getCoMPos().x();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("FootholdOptimization"),
                       foot << " nominal pos: "
                            << footholds_nominal_pos[foot].transpose());
  }

  nf = foot_names.size();

  vel_cmd.setZero();
  yawd_ = 0.0;
}

FootholdOptimization::~FootholdOptimization() {}

void FootholdOptimization::generate_reference() {
  const scalar_t t = nodeHandle_->now().seconds();

  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);
  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  auto mode_schedule = refTrajBuffer_->get_mode_schedule();

  if (mode_schedule == nullptr)
    return;

  vector_t rpy_m, rpy_dot_m;
  vector_t pos_m, vel_m;

  auto pos_traj_ref = refTrajBuffer_->get_base_pos_traj();
  auto rpy_traj_ref = refTrajBuffer_->get_base_rpy_traj();

  rpy_m = toEulerAngles(base_pose_m.rotation());
  rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
  pos_m = base_pose_m.translation();
  vel_m = base_twist.linear();

  if (pos_traj_ref == nullptr || rpy_traj_ref == nullptr) {
    rpy_m = toEulerAngles(base_pose_m.rotation());
    rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
    pos_m = base_pose_m.translation();
    vel_m = base_twist.linear();
  } else {
    rpy_m = rpy_traj_ref->evaluate(t);
    rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
    vector_t rpy_c = toEulerAngles(base_pose_m.rotation());
    if ((rpy_c - rpy_m).norm() > 0.1) {
      rpy_m = 0.1 * (rpy_m - rpy_c).normalized() + rpy_c;
    }
    pos_m = pos_traj_ref->evaluate(t);
    // vector_t pos_c = base_pose_m.translation();
    // if ((pos_c - pos_m).norm() > 0.03) {
    //   pos_m = 0.03 * (pos_m - pos_c).normalized() + pos_c;
    // }
    vel_m = base_twist.linear();
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> rpy_t, pos_t;
  scalar_t horizon_time = mode_schedule->duration();
  if (vel_cmd.norm() < 0.05) {
    scalar_t zd = 0.636;
    scalar_t mod_z = 0.0;
    time.emplace_back(t);
    time.emplace_back(t + 0.5 * horizon_time);
    time.emplace_back(t + horizon_time);
    rpy_t.emplace_back(vector3_t(0, 0, 0));
    rpy_t.emplace_back(vector3_t(0, 0, 0));
    rpy_t.emplace_back(vector3_t(0, 0, 0));
    pos_t.emplace_back(vector3_t(0.0, 0.0, zd - mod_z * (0.05 * sin(6 * t))));
    pos_t.emplace_back(vector3_t(0.0, 0.0, zd - mod_z * (0.05 * sin(6 * t))));
    pos_t.emplace_back(vector3_t(0.0, 0.0, zd - mod_z * (0.05 * sin(6 * t))));
  } else {
    size_t N = horizon_time / 0.05;
    vector3_t vel_des;
    vel_des << vel_cmd.x(), vel_cmd.y(), vel_cmd.z();
    vel_des = base_pose_m.rotation() * vel_des;

    scalar_t h_des = 0.32;
    if (0.55 > h_des || h_des > 0.65) {
      h_des = 0.636;
      ;
    }
    rpy_m.head(2).setZero();
    for (size_t k = 0; k < N; k++) {
      time.push_back(t + 0.05 * k);
      rpy_m.z() += 0.05 * yawd_;
      rpy_t.emplace_back(rpy_m);
      pos_m += 0.05 * vel_des;
      pos_m.z() = h_des;
      pos_t.emplace_back(pos_m);
    }
  }

  /* std::cout << "############# "
            << "base traj des"
            << " ##############\n";
  for (size_t i = 0; i < time.size(); i++) {
    std::cout << " t: " << time[i] - time.front()
              << " pos: " << pos_t[i].transpose() << "\n"
              << " rpy: " << rpy_t[i].transpose() << "\n";
  } */

  pos_traj_ref_ = std::make_shared<CubicSplineTrajectory>(3);
  pos_traj_ref_->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  pos_traj_ref_->fit(time, pos_t);

  rpy_traj_ref_ = std::make_shared<CubicSplineTrajectory>(3);
  rpy_traj_ref_->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  rpy_traj_ref_->fit(time, rpy_t);
  refTrajBuffer_->set_base_rpy_traj(rpy_traj_ref_);
}

void FootholdOptimization::optimize() {
  generate_reference();

  footholds_ = std::map<std::string, std::pair<scalar_t, vector3_t>>();

  auto mode_schedule = refTrajBuffer_->get_mode_schedule();
  if (mode_schedule.get()) {
    heuristic();
  }
  refTrajBuffer_->set_footholds(footholds_);
  for (const auto &foot : foot_names) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("FootholdOptimization"),
                       foot << " opt pos: "
                            << footholds_[foot].second.transpose());
  }
}

void FootholdOptimization::heuristic() {

  const scalar_t t = nodeHandle_->now().seconds();
  auto mode_schedule = refTrajBuffer_->get_mode_schedule();

  vector3_t v_des = pos_traj_ref_->derivative(t, 1);

  const auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  const auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  vector3_t rpy_dot =
      getJacobiFromOmegaToRPY(toEulerAngles(base_pose.rotation())) *
      base_twist.angular();

  auto swtr = legged_robot::getTimeOfNextTouchDown(0, mode_schedule);

  std::cout << "swtr: " << swtr[0] << ", " << swtr[1] << "\n";

  const scalar_t p_rel_x_max = 0.5f;
  const scalar_t p_rel_y_max = 0.4f;

  for (size_t i = 0; i < nf; i++) {
    const auto &foot_name = foot_names[i];
    const scalar_t nextStanceTime = swtr[i];

    vector3_t pYawCorrected =
        toRotationMatrix(rpy_traj_ref_->evaluate(t + nextStanceTime)) *
        footholds_nominal_pos[foot_name];

    scalar_t pfx_rel, pfy_rel;
    std::pair<scalar_t, vector3_t> foothold;
    foothold.first = nextStanceTime + t;
    foothold.second =
        base_pose.translation() +
        (pYawCorrected + std::max(0.0, nextStanceTime) * vel_cmd);
    pfx_rel = 0.5 * mode_schedule->duration() * v_des.x() +
              0.1 * (base_twist.linear().x() - v_des.x()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (base_twist.linear().y() * rpy_dot.z());
    pfy_rel = 0.5 * mode_schedule->duration() * v_des.y() +
              0.1 * (base_twist.linear().y() - v_des.y()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (-base_twist.linear().x() * rpy_dot.z());
    pfx_rel = std::min(std::max(pfx_rel, -p_rel_x_max), p_rel_x_max);
    pfy_rel = std::min(std::max(pfy_rel, -p_rel_y_max), p_rel_y_max);
    foothold.second.x() += pfx_rel;
    foothold.second.y() += pfy_rel;
    foothold.second.z() = 0.02;
    footholds_[foot_name] = foothold;
  }
}

void FootholdOptimization::setVelCmd(vector3_t vd, scalar_t yawd) {
  vel_cmd = vd;
  yawd_ = yawd;
}

} // namespace clear