#include "generation/TrajectorGeneration.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

TrajectorGeneration::TrajectorGeneration(string config_yaml)
    : Node("TrajectorGeneration") {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string estimated_state_topic =
      config_["global"]["topic_names"]["estimated_states"].as<std::string>();
  std::string mode_schedule_topic =
      config_["global"]["topic_names"]["mode_schedule"].as<std::string>();
  std::string trajectories_topic =
      config_["global"]["topic_names"]["trajectories"].as<std::string>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  estimated_state_subscription_ =
      this->create_subscription<trans::msg::EstimatedStates>(
          topic_prefix + estimated_state_topic, qos,
          std::bind(&TrajectorGeneration::estimated_state_callback, this,
                    std::placeholders::_1));
  mode_schedule_subscription_ =
      this->create_subscription<trans::msg::ModeScheduleTrans>(
          topic_prefix + mode_schedule_topic, qos,
          std::bind(&TrajectorGeneration::mode_schedule_callback, this,
                    std::placeholders::_1));
  trajectories_pub_ptr_ = this->create_publisher<trans::msg::TrajectoryArray>(
      topic_prefix + trajectories_topic, qos);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(this->get_logger(), "model file: %s", urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  auto foot_names =
      config_["model"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    RCLCPP_INFO(this->get_logger(), "foot name: %s", name.c_str());
  }
  pinocchioInterface_ptr_->setContactPoints(foot_names);
  base_name = config_["model"]["base_name"].as<std::string>();
  RCLCPP_INFO(this->get_logger(), "base name: %s", base_name.c_str());

  freq_ = config_["generation"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(this->get_logger(), "frequency: %f", freq_);

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

  run_.push(true);
  inner_loop_thread_ = std::thread(&TrajectorGeneration::inner_loop, this);
}

TrajectorGeneration::~TrajectorGeneration() {
  run_.push(false);
  inner_loop_thread_.join();
}

void TrajectorGeneration::inner_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (estimated_state_buffer.get().get() == nullptr ||
        mode_schedule_buffer.get().get() == nullptr) {
      RCLCPP_INFO(this->get_logger(), "skip for waitting message");
    } else {
      updatePinocchioInterface();

      generate_base_traj();

      generate_footholds();

      generate_foot_traj();

      publish_traj();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(this->get_logger(), "max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void TrajectorGeneration::publish_traj() {
  trans::msg::TrajectoryArray msgs;
  // base
  msgs.array.emplace_back(*base_pos_msg);
  msgs.array.emplace_back(*base_rpy_msg);

  // foot
  for (auto &msg : foot_pos_msg) {
    msgs.array.emplace_back(*msg.second);
  }
  trajectories_pub_ptr_->publish(msgs);
}

void TrajectorGeneration::updatePinocchioInterface() {
  auto state = estimated_state_buffer.get();
  const auto &model = pinocchioInterface_ptr_->getModel();

  vector_t qpos(pinocchioInterface_ptr_->nq());
  vector_t qvel(pinocchioInterface_ptr_->nv());

  const auto &joints_state = state->joint_states;
  for (size_t k = 0; k < joints_state.name.size(); k++) {
    if (static_cast<int>(k) < model.njoints &&
        model.existJointName(joints_state.name[k])) {
      pin::Index id = model.getJointId(joints_state.name[k]) - 2;
      qpos[id + 7] = joints_state.position[k];
      qvel[id + 6] = joints_state.velocity[k];
    }
  }

  const auto &odom = state->odom;
  const auto &orientation = odom.pose.pose.orientation;
  const auto &position = odom.pose.pose.position;
  const auto &vel = odom.twist.twist.linear;
  const auto &ang_vel = odom.twist.twist.angular;

  Eigen::Quaternion<scalar_t> quat(orientation.w, orientation.x, orientation.y,
                                   orientation.z);
  matrix3_t rot = quat.toRotationMatrix();
  qpos.head(3) << position.x, position.y, position.z;
  qvel.head(3) << vel.x, vel.y, vel.z;
  qvel.head(3) = rot.transpose() * qvel.head(3);
  qpos.segment(3, 4) << orientation.x, orientation.y, orientation.z,
      orientation.w;
  qvel.segment(3, 3) << ang_vel.x, ang_vel.y, ang_vel.z;
  pinocchioInterface_ptr_->updateRobotState(qpos, qvel);

  auto stance_leg = quadruped::modeNumber2StanceLeg(
      mode_schedule_buffer.get()->getModeFromPhase(0.0));
  std::vector<bool> mask;
  for (auto &flag : stance_leg) {
    mask.push_back(flag);
  }
  if (mask.size() != pinocchioInterface_ptr_->nc()) {
    throw std::runtime_error("mask size is not equal to the "
                             "number of contact points");
  }
  pinocchioInterface_ptr_->setContactMask(mask);
}

void TrajectorGeneration::TrajectorGeneration::generate_base_traj() {
  vector3_t rpy_m, rpy_dot_m;
  vector3_t pos_m, vel_m;
  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);
  rpy_m = toEulerAngles(base_pose_m.rotation());
  pos_m = base_pose_m.translation();

  geometry_msgs::msg::Vector3 pos0, rpy0;
  if (base_pos_msg.get() != nullptr) {
    pos0 = base_pos_msg->knots[0];
  } else {
    // pos0.x = base_pose_m.translation().x();
    // pos0.y = base_pose_m.translation().y();
    // pos0.z = base_pose_m.translation().z();
    pos0.x = 0.0;
    pos0.y = 0.0;
    pos0.z = 0.38;
  }
  if (base_rpy_msg.get() != nullptr) {
    rpy0 = base_rpy_msg->knots[0];
  } else {
    // rpy0.x = rpy_m.x();
    // rpy0.y = rpy_m.y();
    // rpy0.z = rpy_m.z();
    rpy0.x = 0.0;
    rpy0.y = 0.0;
    rpy0.z = 0.0;
  }
  const scalar_t t_now = this->now().seconds();
  const scalar_t dur = mode_schedule_buffer.get()->duration();

  base_pos_msg = std::make_shared<trans::msg::Trajectory>();
  base_pos_msg->header.stamp = this->now();
  base_pos_msg->header.frame_id = base_name + "_pos";
  base_rpy_msg = std::make_shared<trans::msg::Trajectory>();
  base_rpy_msg->header.stamp = this->now();
  base_rpy_msg->header.frame_id = base_name + "_rpy";

  base_pos_msg->time_array.emplace_back(t_now);
  base_pos_msg->time_array.emplace_back(t_now + 0.5 * dur);
  base_pos_msg->time_array.emplace_back(t_now + dur);
  base_pos_msg->knots.emplace_back(pos0);
  base_pos_msg->knots.emplace_back(pos0);
  base_pos_msg->knots.emplace_back(pos0);

  base_rpy_msg->time_array.emplace_back(t_now);
  base_rpy_msg->time_array.emplace_back(t_now + 0.5 * dur);
  base_rpy_msg->time_array.emplace_back(t_now + dur);
  base_rpy_msg->knots.emplace_back(rpy0);
  base_rpy_msg->knots.emplace_back(rpy0);
  base_rpy_msg->knots.emplace_back(rpy0);
}

void TrajectorGeneration::generate_footholds() {

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
    foothold.first = nextStanceTime + this->now().seconds();
    foothold.second = base_pose.translation() +
                      (pYawCorrected + std::max(0.0, nextStanceTime) * v_des);
    pfx_rel = nextStanceTime * v_des.x() +
              0.03 * (base_twist.linear().x() - v_des.x()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (base_twist.linear().y() * rpy_dot.z());
    pfy_rel = nextStanceTime * v_des.y() +
              0.03 * (base_twist.linear().y() - v_des.y()) +
              (0.5 * base_pose.translation().z() / 9.81) *
                  (-base_twist.linear().x() * rpy_dot.z());
    pfx_rel = std::min(std::max(pfx_rel, -p_rel_x_max), p_rel_x_max);
    pfy_rel = std::min(std::max(pfy_rel, -p_rel_y_max), p_rel_y_max);
    foothold.second.x() += pfx_rel;
    foothold.second.y() += pfy_rel;
    foothold.second.z() = 0.023;
    footholds[foot_name] = foothold;
  }

  for (const auto &foothold : footholds) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       foothold.first
                           << " ts="
                           << this->now().seconds() - foothold.second.first
                           << " pos=" << foothold.second.second.transpose()
                           << "\n");
  }
}

void TrajectorGeneration::generate_foot_traj() {
  const auto &foot_names = pinocchioInterface_ptr_->getContactPoints();
  auto mode_schedule = mode_schedule_buffer.get();
  const scalar_t t = this->now().seconds();
  if (xf_start_.empty()) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      const auto &foot_name = foot_names[k];
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      std::pair<scalar_t, vector3_t> xs;
      xs.first = t;
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
      xe.first = t + mode_schedule.get()->duration();
      xe.second = pos;
      xf_end_[foot_name] = std::move(xe);
    }
  }

  auto contact_flag =
      quadruped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(0.0));
  for (size_t k = 0; k < foot_names.size(); k++) {
    const auto &foot_name = foot_names[k];
    vector3_t pos =
        pinocchioInterface_ptr_->getFramePose(foot_name).translation();
    if (contact_flag_[k] != contact_flag[k]) {
      std::pair<scalar_t, vector3_t> xs;
      xs.first = t - numeric_traits::limitEpsilon<scalar_t>();
      xs.second = pos;
      xf_start_[foot_name] = std::move(xs);
    }
    if (contact_flag[k]) {
      if (contact_flag_[k] != contact_flag[k]) {
        std::pair<scalar_t, vector3_t> xe;
        xe.first = t + mode_schedule->timeLeftInMode(0.0);
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

    auto msg = std::make_shared<trans::msg::Trajectory>();
    msg->header.stamp = this->now();
    msg->header.frame_id = foot_name;

    msg->time_array.push_back(xf_start_[foot_name].first);
    msg->knots.push_back(vec3dToMsg(xf_start_[foot_name].second));
    msg->time_array.push_back(
        0.5 * (xf_start_[foot_name].first + xf_end_[foot_name].first));
    vector3_t middle_pos =
        0.5 * (xf_start_[foot_name].second + xf_end_[foot_name].second);
    middle_pos.z() += contact_flag[k] ? 0.0 : 0.15;
    msg->knots.push_back(vec3dToMsg(middle_pos));
    msg->time_array.push_back(xf_end_[foot_name].first);
    msg->knots.push_back(vec3dToMsg(xf_end_[foot_name].second));

    // std::cout << "############# " << foot_name << ": " << pos.transpose()
    //           << " ##############\n";
    // for (size_t i = 0; i < msg->time_array.size(); i++) {
    //   std::cout << " t: " << msg->time_array[i] << " pos: " <<
    //   msg->knots[i].x
    //             << " " << msg->knots[i].y << " " << msg->knots[i].z << "\n";
    // }
    foot_pos_msg[foot_name] = std::move(msg);
  }
  contact_flag_ = contact_flag;
}

void TrajectorGeneration::estimated_state_callback(
    const trans::msg::EstimatedStates::SharedPtr msg) const {
  estimated_state_buffer.push(msg);
}

void TrajectorGeneration::mode_schedule_callback(
    const trans::msg::ModeScheduleTrans::SharedPtr msg) const {
  std::vector<scalar_t> event_phases;
  for (const auto &phase : msg->event_phases) {
    event_phases.push_back(phase);
  }
  std::vector<size_t> mode_sequence;
  for (const auto &mode : msg->mode_sequence) {
    mode_sequence.push_back(static_cast<scalar_t>(mode));
  }
  auto mode_schedule = std::make_shared<ModeSchedule>(
      static_cast<scalar_t>(msg->duration), event_phases, mode_sequence);
  mode_schedule_buffer.push(mode_schedule);

  //   mode_schedule->print();
}

void TrajectorGeneration::extract_foot_switch_info() {
  auto mode_schedule = mode_schedule_buffer.get();
  foot_switch_phase_.clear();

  auto contact_flag_last = quadruped::modeNumber2StanceLeg(
      mode_schedule->getModeFromPhase(mode_schedule->eventPhases().front()));
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

geometry_msgs::msg::Vector3 TrajectorGeneration::vec3dToMsg(vector3_t &vec) {
  geometry_msgs::msg::Vector3 vec_msg;
  vec_msg.x = vec.x();
  vec_msg.y = vec.y();
  vec_msg.z = vec.z();
  return vec_msg;
}

} // namespace clear
