#include "generation/FloatingBaseMotion.h"
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

FloatingBaseMotion::FloatingBaseMotion(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
    std::shared_ptr<ReferenceBuffer> referenceTrajectoriesBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      refTrajBuffer_(referenceTrajectoriesBuffer) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);

  base_name = config_["model"]["base_name"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("FloatingBaseMotion"), "base_name: %s",
              base_name.c_str());
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
}

FloatingBaseMotion::~FloatingBaseMotion() {}

void FloatingBaseMotion::generate() {
  const scalar_t t = nodeHandle_->now().seconds();

  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);
  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  auto mode_schedule = refTrajBuffer_->get_mode_schedule();

  if (mode_schedule == nullptr)
    return;

  vector_t rpy_m, rpy_dot_m;
  vector_t pos_m, vel_m;

  auto base_pos_traj = refTrajBuffer_->get_base_pos_traj();
  auto base_rpy_traj = refTrajBuffer_->get_base_rpy_traj();

  rpy_m = toEulerAngles(base_pose_m.rotation());
  rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
  pos_m = base_pose_m.translation();
  vel_m = base_twist.linear();

  if (base_pos_traj == nullptr || base_rpy_traj == nullptr) {
    rpy_m = toEulerAngles(base_pose_m.rotation());
    rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
    pos_m = base_pose_m.translation();
    vel_m = base_twist.linear();
  } else {
    rpy_m = base_rpy_traj->evaluate(t);
    rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
    vector_t rpy_c = toEulerAngles(base_pose_m.rotation());
    if ((rpy_c - rpy_m).norm() > 0.1) {
      rpy_m = 0.1 * (rpy_m - rpy_c).normalized() + rpy_c;
    }
    pos_m = base_pos_traj->evaluate(t);
    vector_t pos_c = base_pose_m.translation();
    if ((pos_c - pos_m).norm() > 0.03) {
      pos_m = 0.03 * (pos_m - pos_c).normalized() + pos_c;
    }
    vel_m = base_twist.linear();
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> rpy_t, pos_t;
  scalar_t horizon_time = mode_schedule->duration();
  vector3_t vel_cmd = vector3_t::Zero();
  scalar_t yawd_ = 0.0;
  if (vel_cmd.norm() < 0.05) {
    scalar_t zd = 0.5;
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
    if (0.3 > h_des || h_des > 0.4) {
      h_des = 0.32;
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

  auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_pos->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_pos->fit(time, pos_t);
  refTrajBuffer_->set_base_pos_traj(cubicspline_pos);

  auto cubicspline_rpy = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_rpy->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_rpy->fit(time, rpy_t);
  refTrajBuffer_->set_base_rpy_traj(cubicspline_rpy);
}

} // namespace clear
