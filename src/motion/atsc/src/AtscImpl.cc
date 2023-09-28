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
  freq_ = config_["controller"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(nodeHandle_->get_logger(), "frequency: %f", freq_);
  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();
  robot_name = config_["model"]["name"].as<std::string>();

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

  base_name = config_["model"]["base_name"].as<std::string>();

  run_.push(true);
  adaptiveGain_ptr_ = std::make_shared<AdaptiveGain>(
      nodeHandle_, pinocchioInterface_ptr_, base_name);
  adapative_gain_thread_ = std::thread(&AtscImpl::adapative_gain_loop, this);
  wbcPtr_ = std::make_shared<WholeBodyController>(nodeHandle_, config_yaml);
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

void AtscImpl::publishCmds() {
  const auto &model = pinocchioInterface_ptr_->getModel();

  trans::msg::ActuatorCmds msg;
  msg.header.frame_id = robot_name;
  msg.header.stamp = nodeHandle_->now();

  if (actuator_commands_.get() == nullptr) {
    return;
  }

  vector_t torque_mpc = vector_t::Zero(pinocchioInterface_ptr_->na());
  if (feedback_gain_buffer_.get().get() != nullptr) {
    const auto foot_names = pinocchioInterface_ptr_->getContactPoints();
    size_t nf = foot_names.size();
    matrix_t J = matrix_t::Zero(3 * nf, pinocchioInterface_ptr_->nv());

    for (size_t i = 0; i < nf; i++) {
      matrix6x_t Ji;
      pinocchioInterface_ptr_->getJacobia_localWorldAligned(foot_names[i], Ji);
      J.middleRows(3 * i, 3) = Ji.topRows(3);
    }

    vector_t x0(12);
    auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
    auto base_twist =
        pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
    vector3_t rpy = toEulerAngles(base_pose.rotation());
    x0 << base_pose.translation(), base_twist.linear(), rpy,
        base_twist.angular();
    auto feedback_law = feedback_gain_buffer_.get();
    vector_t par = -J.transpose() * (feedback_law->K * x0 + feedback_law->b) +
                   pinocchioInterface_ptr_->nle();
    torque_mpc = par.tail(pinocchioInterface_ptr_->na());
  }

  for (const auto &joint_name : actuated_joints_name) {
    if (model.existJointName(joint_name)) {
      msg.names.emplace_back(joint_name);
      pin::Index id = model.getJointId(joint_name) - 2;
      msg.gain_p.emplace_back(actuator_commands_->Kp(id));
      msg.pos_des.emplace_back(actuator_commands_->pos(id));
      msg.gaid_d.emplace_back(actuator_commands_->Kd(id));
      msg.vel_des.emplace_back(actuator_commands_->vel(id));
      msg.feedforward_torque.emplace_back(torque_mpc(id));
    }
  }
  actuators_cmds_pub_ptr_->publish(msg);
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

      wbcPtr_->update_state(qpos_ptr, qvel_ptr);
      wbcPtr_->update_trajectory_reference(refTrajPtrBuffer_.get());
      wbcPtr_->update_mode(mode_schedule_buffer.get()->getModeFromPhase(0.0));
      wbcPtr_->update_base_policy(feedback_gain_buffer_.get());
      actuator_commands_ = wbcPtr_->optimize();

      publishCmds();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(
      nodeHandle_->get_logger(), "TSC: max time %f ms,  average time %f ms",
      timer_.getMaxIntervalInMilliseconds(), timer_.getAverageInMilliseconds());
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

} // namespace clear
