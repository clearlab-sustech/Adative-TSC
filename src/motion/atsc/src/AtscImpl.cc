#include "atsc/AtscImpl.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

AtscImpl::AtscImpl(Node::SharedPtr nodeHandle, const std::string config_yaml)
    : nodeHandle_(nodeHandle) {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string actuators_cmds_topic =
      config_["global"]["topic_names"]["actuators_cmds"].as<std::string>();
  std::string mode_schedule_topic =
      config_["global"]["topic_names"]["mode_schedule"].as<std::string>();
  freq_ = config_["controller"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "frequency: %f", freq_);
  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();
  std::string robot_name = config_["model"]["name"].as<std::string>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
  actuators_cmds_pub_ptr_ =
      nodeHandle_->create_publisher<trans::msg::ActuatorCmds>(
          topic_prefix + actuators_cmds_topic, qos);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "model file: %s", urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  auto foot_names =
      config_["model"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    RCLCPP_INFO(nodeHandle_->get_logger(), "foot name: %s", name.c_str());
  }
  pinocchioInterface_ptr_->setContactPoints(foot_names);

  tsc_ptr_ = std::make_shared<TaskSpaceControl>(*pinocchioInterface_ptr_);
  // tasks
  regularizationTask = std::make_shared<RegularizationTask>(
      *pinocchioInterface_ptr_, "RegularizationTask");
  tsc_ptr_->addTask(regularizationTask);

  base_name = config_["model"]["base_name"].as<std::string>();
  floatingBaseTask =
      std::make_shared<FloatingBaseTask>(*pinocchioInterface_ptr_, base_name);
  floatingBaseTask->weightMatrix().diagonal().fill(1e2);
  tsc_ptr_->addTask(floatingBaseTask);

  for (const auto &name : foot_names) {
    auto foot_task =
        std::make_shared<TranslationTask>(*pinocchioInterface_ptr_, name);
    foot_task->disable();
    foot_task->Kp().diagonal().fill(500);
    foot_task->Kd().diagonal().fill(60);
    foot_task->weightMatrix().diagonal().fill(1e2);
    tsc_ptr_->addTask(foot_task);
    foot_task_array.push_back(foot_task);
  }

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
  adaptiveGain_ptr_ = std::make_shared<AdaptiveGain>(
      nodeHandle_, pinocchioInterface_ptr_, base_name);
  adapative_gain_thread_ = std::thread(&AtscImpl::adapative_gain_loop, this);
  wbcPtr_ = std::make_shared<WholeBodyController>(nodeHandle_,
                                                  pinocchioInterface_ptr_);
  wbcPtr_->loadTasksSetting();
  inner_loop_thread_ = std::thread(&AtscImpl::inner_loop, this);
}

AtscImpl::~AtscImpl() {
  run_.push(false);
  inner_loop_thread_.join();
  if (adapative_gain_thread_.joinable()) {
    adapative_gain_thread_.join();
  }
}

void AtscImpl::update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                                    std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void AtscImpl::update_trajectory_reference(
    std::shared_ptr<const TrajectoriesArray> referenceTrajectoriesPtr) {
  refTrajPtrBuffer_.push(referenceTrajectoriesPtr);
}

void AtscImpl::update_mode_schedule(
    const std::shared_ptr<ModeSchedule> mode_schedule) {
  mode_schedule_buffer.push(mode_schedule);
}

void AtscImpl::updateFloatingBaseTask() {
  // todo: update
  vector6_t &acc_fb = floatingBaseTask->spatialAccRef();
  auto pos_traj = refTrajPtrBuffer_.get()->get_base_pos_traj();
  auto rpy_traj = refTrajPtrBuffer_.get()->get_base_rpy_traj();
  auto policy = feedback_gain_buffer_.get();
  if (policy.get() != nullptr && pos_traj.get() != nullptr &&
      rpy_traj.get() != nullptr) {
    scalar_t t = nodeHandle_->now().seconds() + 1.0 / freq_;
    vector_t x0(12);
    auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
    auto base_twist =
        pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
    vector_t rpy = toEulerAngles(base_pose.rotation());
    vector3_t rpy_err = compute_euler_angle_err(rpy, rpy_traj->evaluate(t));
    vector3_t omega_des =
        getJacobiFromRPYToOmega(rpy) * rpy_traj->derivative(t, 1);

    x0 << base_pose.translation() - pos_traj->evaluate(t),
        base_twist.linear() - pos_traj->derivative(t, 1), rpy_err,
        pinocchioInterface_ptr_->getData().Ig.inertia() *
            (base_twist.angular() - omega_des);
    acc_fb = policy->K * x0 + policy->b;
    if (acc_fb.norm() > 20) {
      acc_fb = 20.0 * acc_fb.normalized();
    }
    // to local coordinate
    acc_fb.head(3) = base_pose.rotation().transpose() * acc_fb.head(3);
    acc_fb.tail(3) = base_pose.rotation().transpose() * acc_fb.tail(3);
  } else {
    acc_fb.setZero();
  }
  // RCLCPP_INFO_STREAM(nodeHandle_->get_logger(),
  //                    "acc fb: " << acc_fb.transpose() << "\n");
}

void AtscImpl::updateFootSwingBaseTask() {
  // todo:
  auto stance_leg = quadruped::modeNumber2StanceLeg(
      mode_schedule_buffer.get()->getModeFromPhase(0.0));
  auto t = nodeHandle_->now().seconds() + 1.0 / freq_;
  auto foot_traj = refTrajPtrBuffer_.get()->get_foot_pos_traj();
  auto base_pos_traj = refTrajPtrBuffer_.get()->get_base_pos_ref_traj();
  auto foot_names = pinocchioInterface_ptr_->getContactPoints();
  for (size_t i = 0; i < stance_leg.size(); i++) {
    if (stance_leg[i]) {
      foot_task_array[i]->disable();
    } else {
      foot_task_array[i]->enable();
      auto traj = foot_traj[foot_names[i]];
      foot_task_array[i]->posRef() =
          traj->evaluate(t) - base_pos_traj->evaluate(t) +
          pinocchioInterface_ptr_->getFramePose(base_name).translation();
      foot_task_array[i]->velRef() =
          traj->derivative(t, 1) - base_pos_traj->derivative(t, 1) +
          pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name)
              .linear();
      foot_task_array[i]->accRef() = traj->derivative(t, 2);
    }
  }
}

void AtscImpl::publishCmds() {
  const auto &model = pinocchioInterface_ptr_->getModel();
  auto torque = tsc_ptr_->getOptimalTorque();

  for (size_t i = 0; i < actuator_commands_->names.size(); i++) {
    if (static_cast<int>(i) < model.njoints &&
        model.existJointName(actuated_joints_name[i])) {
      pin::Index id = model.getJointId(actuated_joints_name[i]) - 2;
      actuator_commands_->feedforward_torque[i] = torque(i);
      actuator_commands_->names[id] = actuated_joints_name[i];
    } else {
      actuator_commands_->feedforward_torque[i] = 0;
    }
    // actuator_commands_->gain_p[i] = 0.;
    // actuator_commands_->pos_des[i] = 0.;
    // actuator_commands_->gaid_d[i] = 0.;
    // actuator_commands_->vel_des[i] = 0.;
  }
  actuators_cmds_pub_ptr_->publish(*actuator_commands_);
}

void AtscImpl::inner_loop() {

  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr ||
        mode_schedule_buffer.get().get() == nullptr ||
        refTrajPtrBuffer_.get().get() == nullptr) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(int64_t(1000 / freq_)));
    } else {
      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

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

      updateFloatingBaseTask();

      updateFootSwingBaseTask();

      try {
        tsc_ptr_->solve();
      } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
      }
      // if (wbcPtr_.get() != nullptr) {
      //   wbcPtr_->update_trajectory_reference(refTrajPtrBuffer_.get());
      //   wbcPtr_->update_mode(mode_schedule_buffer.get()->getModeFromPhase(0.0));
      //   wbcPtr_->update_base_policy(feedback_gain_buffer_.get());
      //   auto sol = wbcPtr_->optimize();
      // }
      prepare_cmds();

      inverse_kinematics();

      publishCmds();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(
      nodeHandle_->get_logger(), "TSC: max time %f ms,  average time %f ms",
      timer_.getMaxIntervalInMilliseconds(), timer_.getAverageInMilliseconds());
}

void AtscImpl::prepare_cmds() {
  actuator_commands_ = std::make_shared<trans::msg::ActuatorCmds>();
  actuator_commands_->header.frame_id = robot_name;
  actuator_commands_->header.stamp = nodeHandle_->now();
  size_t nj = actuated_joints_name.size();

  actuator_commands_->gain_p.resize(nj);
  actuator_commands_->gaid_d.resize(nj);
  actuator_commands_->pos_des.resize(nj);
  actuator_commands_->vel_des.resize(nj);
  actuator_commands_->feedforward_torque.resize(nj);
  actuator_commands_->names.resize(nj);
}

void AtscImpl::adapative_gain_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(50.0);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr ||
        mode_schedule_buffer.get().get() == nullptr ||
        refTrajPtrBuffer_.get().get() == nullptr) {
      continue;
    } else {
      // RCLCPP_INFO(nodeHandle_->get_logger(), "Adaptive Gain Computaion:
      // run");
      adaptiveGain_ptr_->update_mode_schedule(mode_schedule_buffer.get());
      adaptiveGain_ptr_->update_trajectory_reference(refTrajPtrBuffer_.get());
      feedback_gain_buffer_.push(adaptiveGain_ptr_->compute());
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(nodeHandle_->get_logger(),
              "Adaptive Gain Computaion: max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void AtscImpl::inverse_kinematics() {
  auto foot_traj_array = refTrajPtrBuffer_.get()->get_foot_pos_traj();
  auto base_pos_traj = refTrajPtrBuffer_.get()->get_base_pos_traj();

  if (foot_traj_array.empty() || base_pos_traj.get() == nullptr) {
    return;
  }

  int nj = static_cast<int>(pinocchioInterface_ptr_->na());
  const auto foot_names = pinocchioInterface_ptr_->getContactPoints();
  size_t nf = foot_names.size();
  auto contact_flag = pinocchioInterface_ptr_->getContactMask();
  const scalar_t dt_ = 1.0 / freq_;
  scalar_t time_c = nodeHandle_->now().seconds() + dt_;
  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  for (size_t k = 0; k < nf; k++) {
    const auto &foot_name = foot_names[k];
    matrix6x_t Jac_k;
    pinocchioInterface_ptr_->getJacobia_localWorldAligned(foot_name, Jac_k);
    vector<int> idx;
    for (int i = 0; i < nj; i++) {
      if (Jac_k.col(i + 6).head(3).norm() > 0.01) {
        idx.emplace_back(i);
      }
    }
    if (idx.size() != 3) {
      throw runtime_error("[AtscImpl::inverse_kinematics] idx.size() != 3");
    }

    if (!contact_flag[k]) {
      const auto foot_traj = foot_traj_array[foot_name];
      matrix3_t Js_;
      vector3_t qpos_s;

      for (size_t i = 0; i < 3; i++) {
        Js_.col(i) = Jac_k.col(idx[i] + 6).head(3);
        qpos_s(i) = pinocchioInterface_ptr_->qpos()(7 + idx[i]);
      }
      matrix3_t J_inv = Js_.inverse();

      vector3_t pos_des, vel_des;
      vector3_t pos_m, vel_m;
      pos_m = (pinocchioInterface_ptr_->getFramePose(foot_name).translation() -
               base_pose.translation());
      vel_m =
          (pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(foot_name)
               .linear() -
           base_twist.linear());

      if (base_pos_traj.get() == nullptr) {
        pos_des = (foot_traj->evaluate(time_c) - base_pose.translation());
        vel_des = (foot_traj->derivative(time_c, 1) - base_twist.linear());
      } else {
        pos_des =
            (foot_traj->evaluate(time_c) - base_pos_traj->evaluate(time_c));
        vel_des = (foot_traj->derivative(time_c, 1) -
                   base_pos_traj->derivative(time_c, 1));
      }

      vector3_t pos_err = (pos_des - pos_m);
      if (pos_err.norm() > 0.1) {
        pos_err = 0.1 * pos_err.normalized();
      }
      vector3_t vel_err = (vel_des - vel_m);
      if (vel_err.norm() > 0.5) {
        vel_des = 0.5 * vel_err.normalized() + vel_m;
      }

      scalar_t kp_val, kd_val;
      if (pos_err.norm() < 3e-2) {
        kp_val = 60;
      } else {
        kp_val = 30;
      }
      if (vel_err.norm() < 0.1) {
        kd_val = 2;
      } else {
        kp_val = 1.0;
      }
      vector3_t q_des = J_inv * pos_err + qpos_s;
      vector3_t qd_des = J_inv * vel_des;
      for (size_t i = 0; i < 3; i++) {
        actuator_commands_->gain_p[idx[i]] = kp_val;
        actuator_commands_->gaid_d[idx[i]] = kd_val;
        actuator_commands_->pos_des[idx[i]] = q_des(i);
        actuator_commands_->vel_des[idx[i]] = qd_des(i);
      }
    } else {
      vector_t joint_acc_ =
          tsc_ptr_->getOptimalQacc().tail(pinocchioInterface_ptr_->na());
      vector_t jnt_pos = pinocchioInterface_ptr_->qpos().tail(nj);
      vector_t jnt_vel = pinocchioInterface_ptr_->qvel().tail(nj);
      for (size_t i = 0; i < 3; i++) {
        actuator_commands_->gain_p[idx[i]] = 10.0;
        actuator_commands_->gaid_d[idx[i]] = 0.1;
        actuator_commands_->pos_des[idx[i]] =
            jnt_pos[idx[i]] + jnt_vel[idx[i]] * dt_ +
            0.5 * pow(dt_, 2) * joint_acc_[idx[i]];
        actuator_commands_->vel_des[idx[i]] =
            jnt_vel[idx[i]] + dt_ * joint_acc_[idx[i]];
      }
    }
  }
}

} // namespace clear
