#include "control/TrajectoryStabilization.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <fstream>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

TrajectoryStabilization::TrajectoryStabilization(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string actuators_cmds_topic =
      config_["global"]["topic_names"]["actuators_cmds"].as<std::string>();
  freq_ = config_["controller"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"), "frequency: %f",
              freq_);
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
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"), "model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  base_name = config_["model"]["base_name"].as<std::string>();

  run_.push(true);
  vf_ptr_ = std::make_shared<ConstructVectorField>(nodeHandle_,
                                                   pinocchioInterface_ptr_);
  vector_field_thread_ =
      std::thread(&TrajectoryStabilization::adapative_gain_loop, this);
  wbcPtr_ = std::make_shared<WholeBodyController>(nodeHandle_);
  inner_loop_thread_ = std::thread(&TrajectoryStabilization::innerLoop, this);
}

TrajectoryStabilization::~TrajectoryStabilization() {
  run_.push(false);
  inner_loop_thread_.join();
  if (vector_field_thread_.joinable()) {
    vector_field_thread_.join();
  }
}

void TrajectoryStabilization::updateCurrentState(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectoryStabilization::updateReferenceBuffer(
    std::shared_ptr<ReferenceBuffer> referenceBuffer) {
  referenceBuffer_ = referenceBuffer;
}

trans::msg::ActuatorCmds::SharedPtr TrajectoryStabilization::getCmds() {
  if (actuator_commands_buffer.get() == nullptr) {
    return nullptr;
  }
  auto actuator_commands_ = actuator_commands_buffer.get();

  const auto &model = pinocchioInterface_ptr_->getModel();
  trans::msg::ActuatorCmds::SharedPtr msg =
      std::make_shared<trans::msg::ActuatorCmds>();
  msg->header.frame_id = robot_name;
  msg->header.stamp = nodeHandle_->now();
  for (const auto &joint_name : actuated_joints_name) {
    if (model.existJointName(joint_name)) {
      msg->names.emplace_back(joint_name);
      pin::Index id = model.getJointId(joint_name) - 2;
      msg->gain_p.emplace_back(actuator_commands_->Kp(id));
      msg->pos_des.emplace_back(actuator_commands_->pos(id));
      msg->gaid_d.emplace_back(actuator_commands_->Kd(id));
      msg->vel_des.emplace_back(actuator_commands_->vel(id));
      msg->feedforward_torque.emplace_back(actuator_commands_->torque(id));
    }
  }
  return msg;
}

void TrajectoryStabilization::publishCmds() {
  if (actuator_commands_buffer.get() == nullptr) {
    return;
  }
  auto actuator_commands_ = actuator_commands_buffer.get();

  const auto &model = pinocchioInterface_ptr_->getModel();

  trans::msg::ActuatorCmds msg;
  msg.header.frame_id = robot_name;
  msg.header.stamp = nodeHandle_->now();

  for (const auto &joint_name : actuated_joints_name) {
    if (model.existJointName(joint_name)) {
      msg.names.emplace_back(joint_name);
      pin::Index id = model.getJointId(joint_name) - 2;
      msg.gain_p.emplace_back(actuator_commands_->Kp(id));
      msg.pos_des.emplace_back(actuator_commands_->pos(id));
      msg.gaid_d.emplace_back(actuator_commands_->Kd(id));
      msg.vel_des.emplace_back(actuator_commands_->vel(id));
      msg.feedforward_torque.emplace_back(actuator_commands_->torque(id));
    }
  }
  actuators_cmds_pub_ptr_->publish(msg);
}

void TrajectoryStabilization::innerLoop() {

  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get() == nullptr || qvel_ptr_buffer.get() == nullptr ||
        referenceBuffer_ == nullptr) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(int64_t(1000 / freq_)));
    } else {
      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

      wbcPtr_->updateState(qpos_ptr, qvel_ptr);
      wbcPtr_->updateReferenceBuffer(referenceBuffer_);
      wbcPtr_->updateBaseVectorField(vf_param_buffer_.get());
      actuator_commands_buffer.push(wbcPtr_->optimize());

      publishCmds();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"),
              "max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void TrajectoryStabilization::adapative_gain_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(50.0);
  std::fstream save_state("data_log.txt", std::ios::ate | std::ios::out);

  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get() == nullptr || qvel_ptr_buffer.get() == nullptr ||
        referenceBuffer_.get() == nullptr) {
      continue;
    } else {
      // RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"), "Adaptive Gain Computaion:
      // run");
      vf_ptr_->updateReferenceBuffer(referenceBuffer_);
      vf_param_buffer_.push(vf_ptr_->compute());

      auto base_pos = referenceBuffer_->getIntegratedBasePosTraj();
      auto base_rpy = referenceBuffer_->getIntegratedBaseRpyTraj();
      if (base_pos.get() != nullptr && base_rpy.get() != nullptr) {
        const scalar_t t = nodeHandle_->now().seconds();
        auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
        auto base_twist =
            pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
        save_state << base_pose.translation().transpose() << " "
                   << base_twist.linear().transpose() << " "
                   << base_twist.angular().transpose() << " "
                   << base_pos->evaluate(t).transpose() << " "
                   << base_pos->derivative(t, 1).transpose() << " "
                   << (getJacobiFromRPYToOmega(base_rpy->evaluate(t)) *
                       base_rpy->derivative(t, 1))
                          .transpose()
                   << "\n";
      }
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"),
              "Vector Field Construction: max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
  save_state.close();
}

} // namespace clear
