#include "generation/TrajectorGeneration.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

TrajectorGeneration::TrajectorGeneration(Node::SharedPtr nodeHandle,
                                         string config_yaml)
    : nodeHandle_(nodeHandle) {
  auto config_ = YAML::LoadFile(config_yaml);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "model file: %s", urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  auto foot_names =
      config_["model"]["foot_names"].as<std::vector<std::string>>();

  pinocchioInterface_ptr_->setContactPoints(foot_names);
  base_name = config_["model"]["base_name"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "[TrajectorGeneration]  base name: %s",
              base_name.c_str());

  freq_ = config_["generation"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "[TrajectorGeneration] frequency: %f",
              freq_);

  vector_t qpos, qvel;
  qpos.setZero(pinocchioInterface_ptr_->nq());
  qvel.setZero(pinocchioInterface_ptr_->nv());
  pinocchioInterface_ptr_->updateRobotState(qpos, qvel);
  for (const auto &foot : foot_names) {
    footholds_nominal_pos[foot] =
        pinocchioInterface_ptr_->getFramePose(foot).translation();
    footholds_nominal_pos[foot].z() = 0.0;
  }
  contact_flag_ = {true, true, true, true};

  refTrajBuffer_ = std::make_shared<TrajectoriesArray>();

  run_.push(true);
  inner_loop_thread_ = std::thread(&TrajectorGeneration::inner_loop, this);
}

TrajectorGeneration::~TrajectorGeneration() {
  run_.push(false);
  inner_loop_thread_.join();
}

void TrajectorGeneration::update_current_state(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectorGeneration::update_mode_schedule(
    std::shared_ptr<ModeSchedule> mode_schedule) {
  mode_schedule_buffer.push(mode_schedule);
}

std::shared_ptr<const TrajectoriesArray>
TrajectorGeneration::get_trajectory_reference() {
  return refTrajBuffer_;
}
std::map<std::string, std::pair<scalar_t, vector3_t>>
TrajectorGeneration::get_footholds() {
  return footholds;
}

void TrajectorGeneration::inner_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr ||
        mode_schedule_buffer.get().get() == nullptr) {
      continue;
    } else {
      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

      scalar_t t = nodeHandle_->now().seconds();

      generate_base_traj(t);

      generate_footholds(t);

      generate_foot_traj(t);
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(nodeHandle_->get_logger(),
              "[TrajectorGeneration] max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void TrajectorGeneration::TrajectorGeneration::generate_base_traj(
    scalar_t t_now) {
  vector_t rpy_m, rpy_dot_m;
  vector_t pos_m, vel_m;
  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);
  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  if (refTrajBuffer_->get_base_pos_ref_traj().get() == nullptr ||
      refTrajBuffer_->get_base_rpy_traj().get() == nullptr) {
    rpy_m = toEulerAngles(base_pose_m.rotation());
    rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
    pos_m = base_pose_m.translation();
    vel_m = base_twist.linear();
  } else {
    auto base_pos_traj = refTrajBuffer_->get_base_pos_ref_traj();
    auto base_rpy_traj = refTrajBuffer_->get_base_rpy_traj();

    rpy_m = base_rpy_traj->evaluate(t_now);
    rpy_dot_m = getJacobiFromOmegaToRPY(rpy_m) * base_twist.angular();
    vector_t rpy_c = toEulerAngles(base_pose_m.rotation());
    if ((rpy_c - rpy_m).norm() > 0.1) {
      rpy_m = 0.1 * (rpy_m - rpy_c).normalized() + rpy_c;
    }
    pos_m = base_pos_traj->evaluate(t_now);
    vector_t pos_c = base_pose_m.translation();
    if ((pos_c - pos_m).norm() > 0.03) {
      pos_m = 0.03 * (pos_m - pos_c).normalized() + pos_c;
    }
    vel_m = base_twist.linear();
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> rpy_t, pos_t;
  scalar_t horizon_time = mode_schedule_buffer.get()->duration();
  if (true) {
    scalar_t zd = 0.38;
    scalar_t mod_z = 0.0;
    time.emplace_back(t_now);
    time.emplace_back(t_now + 0.5 * horizon_time);
    time.emplace_back(t_now + horizon_time);
    rpy_t.emplace_back(vector3_t(0, 0, 0));
    rpy_t.emplace_back(vector3_t(0, 0, 0));
    rpy_t.emplace_back(vector3_t(0, 0, 0));
    pos_t.emplace_back(vector3_t(0, 0, zd - mod_z * (0.05 * sin(6 * t_now))));
    pos_t.emplace_back(vector3_t(
        0, 0, zd - mod_z * (0.05 * sin(6 * (t_now + 0.5 * horizon_time)))));
    pos_t.emplace_back(
        vector3_t(0, 0, zd - mod_z * (0.05 * sin(6 * (t_now + horizon_time)))));
  } else {
    // size_t N = horizon_time / 0.05;
    // vector3_t vel_des;
    // vel_des << commands->vel_des.x, commands->vel_des.y, commands->vel_des.z;
    // vel_des = base_pose_m.rotation() * vel_des;

    // scalar_t h_des = commands->height_des;
    // if (0.35 > h_des || h_des > 0.6) {
    //   h_des = 0.48;
    // }
    // rpy_m.head(2).setZero();
    // for (size_t k = 0; k < N; k++) {
    //   time.push_back(t_now + 0.05 * k);
    //   rpy_m.z() += 0.05 * commands->yawdot_des;
    //   rpy_t.emplace_back(rpy_m);
    //   pos_m += 0.05 * vel_des;
    //   pos_m.z() = h_des;
    //   pos_t.emplace_back(pos_m);
    // }
  }

  /* std::cout << "############# "
            << "base traj des"
            << " ##############\n";
  for (size_t i = 0; i < time.size(); i++) {
    std::cout << " t: " << time[i] - time.front()
              << " pos: " << pos_t[i].transpose() << "\n";
  } */

  auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_pos->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_pos->fit(time, pos_t);
  refTrajBuffer_->set_base_pos_ref_traj(cubicspline_pos);
  refTrajBuffer_->set_base_pos_traj(cubicspline_pos);

  /* if (refTrajBuffer_->get_base_pos_traj().get() == nullptr) {
    refTrajBuffer_->set_base_pos_traj(cubicspline_pos);
  } */

  auto cubicspline_rpy = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_rpy->set_boundary(
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero(),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_rpy->fit(time, rpy_t);
  refTrajBuffer_->set_base_rpy_traj(cubicspline_rpy);
}

void TrajectorGeneration::generate_footholds(scalar_t t_now) {

  auto mode_schedule = mode_schedule_buffer.get();
  const auto &foot_names = pinocchioInterface_ptr_->getContactPoints();
  extract_foot_switch_info();

  const auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  const auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  const scalar_t p_rel_x_max = 0.35f;
  const scalar_t p_rel_y_max = 0.3f;

  vector3_t rpy_dot =
      getJacobiFromOmegaToRPY(toEulerAngles(base_pose.rotation())) *
      base_twist.angular();

  vector3_t v_des = vector3_t::Zero();

  for (size_t i = 0; i < foot_names.size(); i++) {
    const auto &foot_name = foot_names[i];
    const scalar_t nextStanceTime =
        foot_switch_phase_[foot_name].front() * mode_schedule->duration();

    vector3_t pYawCorrected = footholds_nominal_pos[foot_name];

    scalar_t pfx_rel, pfy_rel;
    std::pair<scalar_t, vector3_t> foothold;
    foothold.first = nextStanceTime + t_now;
    foothold.second = base_pose.translation() +
                      (pYawCorrected + std::max(0.0, nextStanceTime) * base_twist.linear());
    pfx_rel = 0.5 * nextStanceTime * v_des.x() +
              0.3 * (base_twist.linear().x() - v_des.x()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (base_twist.linear().y() * rpy_dot.z());
    pfy_rel = 0.5 * nextStanceTime * v_des.y() +
              0.3 * (base_twist.linear().y() - v_des.y()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (-base_twist.linear().x() * rpy_dot.z());
    pfx_rel = std::min(std::max(pfx_rel, -p_rel_x_max), p_rel_x_max);
    pfy_rel = std::min(std::max(pfy_rel, -p_rel_y_max), p_rel_y_max);
    foothold.second.x() += pfx_rel;
    foothold.second.y() += pfy_rel;
    foothold.second.z() = 0.023;
    footholds[foot_name] = foothold;
  }

  // for (const auto &foothold : footholds) {
  //   RCLCPP_INFO_STREAM(nodeHandle_->get_logger(),
  //                      foothold.first
  //                          << " ts="
  //                          << nodeHandle_->now().seconds() -
  //                          foothold.second.first
  //                          << " pos=" << foothold.second.second.transpose()
  //                          << "\n");
  // }
}

void TrajectorGeneration::generate_foot_traj(scalar_t t_now) {
  const auto &foot_names = pinocchioInterface_ptr_->getContactPoints();
  auto mode_schedule = mode_schedule_buffer.get();
  if (xf_start_.empty()) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      const auto &foot_name = foot_names[k];
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      std::pair<scalar_t, vector3_t> xs;
      xs.first = t_now;
      xs.second = pos;
      xf_start_[foot_name] = std::move(xs);
    }
  }
  if (xf_end_.empty()) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      const auto &foot_name = foot_names[k];
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      std::pair<scalar_t, vector3_t> xe;
      xe.first = t_now + mode_schedule.get()->duration();
      xe.second = pos;
      xf_end_[foot_name] = std::move(xe);
    }
  }
  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>> foot_pos_traj;

  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(0.0));
  for (size_t k = 0; k < foot_names.size(); k++) {
    const auto &foot_name = foot_names[k];
    vector3_t pos =
        pinocchioInterface_ptr_->getFramePose(foot_name).translation();
    if (contact_flag_[k] != contact_flag[k]) {
      std::pair<scalar_t, vector3_t> xs;
      xs.first = t_now - numeric_traits::limitEpsilon<scalar_t>();
      xs.second = pos;
      xf_start_[foot_name] = std::move(xs);
    }
    if (contact_flag[k]) {
      if (contact_flag_[k] != contact_flag[k]) {
        std::pair<scalar_t, vector3_t> xe;
        xe.first = t_now + mode_schedule->timeLeftInMode(0.0);
        xe.second = pos;
        xf_end_[foot_name] = std::move(xe);
      }
    } else {
      std::pair<scalar_t, vector3_t> xe;
      xe.first = footholds[foot_name].first;
      xe.second = footholds[foot_name].second;
      xf_end_[foot_name] = std::move(xe);
    }
    if (xf_start_[foot_name].first <
        xf_end_[foot_name].first - mode_schedule.get()->duration()) {
      xf_start_[foot_name].first =
          xf_end_[foot_name].first - mode_schedule.get()->duration();
      xf_start_[foot_name].second = pos;
    }

    std::vector<scalar_t> time;
    std::vector<vector_t> pos_t;
    time.push_back(xf_start_[foot_name].first);
    pos_t.push_back(xf_start_[foot_name].second);
    time.push_back(0.5 *
                   (xf_start_[foot_name].first + xf_end_[foot_name].first));
    vector3_t middle_pos =
        0.5 * (xf_start_[foot_name].second + xf_end_[foot_name].second);
    middle_pos.z() += contact_flag[k] ? 0.0 : 0.15;
    pos_t.push_back(middle_pos);
    time.push_back(xf_end_[foot_name].first);
    pos_t.push_back(xf_end_[foot_name].second);

    /* std::cout << "############# " << foot_name << ": " << pos.transpose()
              << " ##############\n";
    for (size_t i = 0; i < time.size(); i++) {
      std::cout << " t: " << time[i] - time.front()
                << " pos: " << pos_t[i].transpose() << "\n";
    } */

    auto cubicspline_ = std::make_shared<CubicSplineTrajectory>(
        3, CubicSplineInterpolation::SplineType::cspline);
    cubicspline_->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero(),
        CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
    cubicspline_->fit(time, pos_t);
    /* std::cout << "############# opt " << foot_name << " ##############\n";
    for (size_t i = 0; i < time.size(); i++) {
      std::cout
          << " t: " << time[i] - time.front() << " pos: "
          << foot_traj_ptr->pos_trajectoryPtr->evaluate(time[i]).transpose()
          << "\n";
    } */
    foot_pos_traj[foot_name] = std::move(cubicspline_);
  }
  contact_flag_ = contact_flag;
  refTrajBuffer_->set_foot_pos_traj(foot_pos_traj);
}

void TrajectorGeneration::extract_foot_switch_info() {
  auto mode_schedule = mode_schedule_buffer.get();
  foot_switch_phase_.clear();

  auto contact_flag_last =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(0.0));
  const auto foot_names = pinocchioInterface_ptr_->getContactPoints();
  for (const auto &phase : mode_schedule->eventPhases()) {
    auto contact_flag =
        quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
    for (size_t k = 0; k < foot_names.size(); k++) {
      if (contact_flag[k] && contact_flag[k] != contact_flag_last[k]) {
        foot_switch_phase_[foot_names[k]].emplace_back(phase);
      }
    }
    contact_flag_last = contact_flag;
  }
  if (foot_switch_phase_.size() == 0) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      foot_switch_phase_[foot_names[k]].emplace_back(1.0);
    }
  } else {
    for (size_t k = 0; k < foot_names.size(); k++) {
      if (foot_switch_phase_[foot_names[k]].empty()) {
        foot_switch_phase_[foot_names[k]].emplace_back(1.0);
      }
    }
  }
}

} // namespace clear
