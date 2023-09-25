#include "AtscImpl.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <yaml-cpp/yaml.h>

namespace clear {

AtscImpl::AtscImpl(const std::string config_yaml) : Node("AdaptiveCtrl") {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string estimated_state_topic =
      config_["global"]["topic_names"]["estimated_states"].as<std::string>();
  std::string actuators_cmds_topic =
      config_["global"]["topic_names"]["actuators_cmds"].as<std::string>();
  std::string mode_schedule_topic =
      config_["global"]["topic_names"]["mode_schedule"].as<std::string>();
  std::string trajectories_topic =
      config_["global"]["topic_names"]["trajectories"].as<std::string>();
  dt_ = config_["controller"]["dt"].as<scalar_t>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  estimated_state_subscription_ =
      this->create_subscription<trans::msg::EstimatedStates>(
          topic_prefix + estimated_state_topic, qos,
          std::bind(&AtscImpl::estimated_state_callback, this,
                    std::placeholders::_1));
  mode_schedule_subscription_ =
      this->create_subscription<trans::msg::ModeScheduleTrans>(
          topic_prefix + mode_schedule_topic, qos,
          std::bind(&AtscImpl::mode_schedule_callback, this,
                    std::placeholders::_1));
  trajectories_subscription_ =
      this->create_subscription<trans::msg::TrajectoryArray>(
          topic_prefix + trajectories_topic, qos,
          std::bind(&AtscImpl::trajectories_callback, this,
                    std::placeholders::_1));
  actuators_cmds_pub_ptr_ = this->create_publisher<trans::msg::ActuatorCmds>(
      topic_prefix + actuators_cmds_topic, qos);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(this->get_logger(), "[AtscQuadruped] model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  auto foot_names =
      config_["model"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    RCLCPP_INFO(this->get_logger(), "[AtscQuadruped] foot name: %s",
                name.c_str());
  }
  pinocchioInterface_ptr_->setContactPoints(foot_names);

  tsc_ptr_ = std::make_shared<TaskSpaceControl>(*pinocchioInterface_ptr_);
  // tasks
  regularizationTask = std::make_shared<RegularizationTask>(
      *pinocchioInterface_ptr_, "RegularizationTask");
  tsc_ptr_->addTask(regularizationTask);

  base_name = config_["model"]["base_name"].as<std::string>();
  floatingBaseTask =
      std::make_shared<SE3MotionTask>(*pinocchioInterface_ptr_, base_name);
  floatingBaseTask->weightMatrix().diagonal().fill(1e2);
  floatingBaseTask->Kp().diagonal().fill(100);
  floatingBaseTask->Kd().diagonal().fill(20);
  tsc_ptr_->addTask(floatingBaseTask);

  // constraints
  ne_eq = std::make_shared<NewtonEulerEq>(*pinocchioInterface_ptr_,
                                          "NewtonEulerEq");
  tsc_ptr_->addLinearConstraint(ne_eq);

  maintainContact = std::make_shared<ContactPointsConstraints>(
      *pinocchioInterface_ptr_, "ContactPointsConstraints");
  tsc_ptr_->addLinearConstraint(maintainContact);

  frictionCone = std::make_shared<ContactForceConstraints>(
      *pinocchioInterface_ptr_, "ContactForceConstraints");
  tsc_ptr_->addLinearConstraint(frictionCone);

  torqueLimit = std::make_shared<ActuatorLimit>(*pinocchioInterface_ptr_,
                                                "ActuatorLimit");
  tsc_ptr_->addLinearConstraint(torqueLimit);

  run_.push(true);
  inner_loop_thread_ = std::thread(&AtscImpl::inner_loop, this);
}

AtscImpl::~AtscImpl() {
  run_.push(false);
  inner_loop_thread_.join();
  adapative_gain_thread_.join();
}

void AtscImpl::estimated_state_callback(
    const trans::msg::EstimatedStates::SharedPtr msg) const {
  estimated_state_buffer.push(msg);
}

void AtscImpl::mode_schedule_callback(
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

  mode_schedule->print();
}

void AtscImpl::trajectories_callback(
    const trans::msg::TrajectoryArray::SharedPtr msg) const {
  trajectories_msg_buffer_.push(msg);
  trajectories_updated_.push(true);
}

void AtscImpl::updateTask() {
  // todo: update
  floatingBaseTask->SE3Ref().translation() << 0, 0, 0.4;
  floatingBaseTask->SE3Ref().rotation().setIdentity();
  floatingBaseTask->spatialVelRef().setZero();
  floatingBaseTask->spatialAccRef().setZero();
}

void AtscImpl::trajectoriesPreprocessing() {
  if (trajectories_updated_.get()) {
    trajectories_updated_.push(false);
    auto trajectories_ptr = trajectories_msg_buffer_.get();
  }
}

void AtscImpl::updatePinocchioInterface() {
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

void AtscImpl::publishCmds() {
  auto state = estimated_state_buffer.get();
  const auto &model = pinocchioInterface_ptr_->getModel();
  trans::msg::ActuatorCmds actuatorCmds;
  actuatorCmds.header.frame_id = state->odom.header.frame_id;
  actuatorCmds.header.stamp = this->now();
  actuatorCmds.names = state->joint_states.name;

  auto torque = tsc_ptr_->getOptimalTorque();
  for (size_t i = 0; i < actuatorCmds.names.size(); i++) {
    if (static_cast<int>(i) < model.njoints &&
        model.existJointName(actuatorCmds.names[i])) {
      pin::Index id = model.getJointId(actuatorCmds.names[i]) - 2;
      actuatorCmds.feedforward_torque.emplace_back(torque(id));
    } else {
      actuatorCmds.feedforward_torque.emplace_back(0);
    }
    actuatorCmds.gain_p.emplace_back(0);
    actuatorCmds.pos_des.emplace_back(0);
    actuatorCmds.gaid_d.emplace_back(0);
    actuatorCmds.vel_des.emplace_back(0);
  }
  actuators_cmds_pub_ptr_->publish(actuatorCmds);
}

void AtscImpl::inner_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(1.0 / dt_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (estimated_state_buffer.get().get() == nullptr ||
        mode_schedule_buffer.get().get() == nullptr ||
        trajectories_msg_buffer_.get().get() == nullptr) {
      RCLCPP_INFO(this->get_logger(), "TSC: wait for message");
    } else {
      trajectoriesPreprocessing();

      updatePinocchioInterface();

      updateTask();

      tsc_ptr_->solve();

      publishCmds();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(this->get_logger(), "TSC: max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void AtscImpl::adapative_gain_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(50.0);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (estimated_state_buffer.get().get() == nullptr ||
        mode_schedule_buffer.get().get() == nullptr ||
        refTrajPtrBuffer_.get().get() == nullptr) {
      RCLCPP_INFO(this->get_logger(),
                  "Adaptive Gain Computaion: wait for message");
    } else {
      adaptiveGain_ptr_->update_mode_schedule(mode_schedule_buffer.get());
      adaptiveGain_ptr_->update_trajectory_reference(refTrajPtrBuffer_.get());
      feedback_gain_buffer_.push(adaptiveGain_ptr_->compute());
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(this->get_logger(),
              "Adaptive Gain Computaion: max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void AtscImpl::enable_adaptive_gain() {
  adaptiveGain_ptr_ = std::make_shared<AdaptiveGain>(
      this->shared_from_this(), pinocchioInterface_ptr_, base_name);
  adapative_gain_thread_ = std::thread(&AtscImpl::adapative_gain_loop, this);
}

} // namespace clear
