#include "atsc/TrajectoryStabilization.h"
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
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"), "frequency: %f", freq_);

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
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  run_.push(true);
  vf_construct_ptr_ = std::make_shared<ConstructVectorField>(nodeHandle_, pinocchioInterface_ptr_);
  vector_field_thread_ = std::thread(&TrajectoryStabilization::vector_field_loop, this);
  wbcPtr_ = std::make_shared<WholeBodyController>(nodeHandle_, pinocchioInterface_ptr_);
  inner_loop_thread_ = std::thread(&TrajectoryStabilization::inner_loop, this);
}

TrajectoryStabilization::~TrajectoryStabilization() {
  run_.push(false);
  inner_loop_thread_.join();
  if (vector_field_thread_.joinable()) {
    vector_field_thread_.join();
  }
}

void TrajectoryStabilization::update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                                    std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectoryStabilization::update_trajectory_reference(
    std::shared_ptr<ReferenceBuffer> referenceTrajectoriesPtr) {
  refTrajPtrBuffer_.push(referenceTrajectoriesPtr);
}

void TrajectoryStabilization::publishCmds() {
  const auto &model = pinocchioInterface_ptr_->getModel();

  trans::msg::ActuatorCmds msg;
  msg.header.frame_id = robot_name;
  msg.header.stamp = nodeHandle_->now();

  if (actuator_commands_.get() == nullptr) {
    return;
  }

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

void TrajectoryStabilization::inner_loop() {

  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);
  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr ||
        refTrajPtrBuffer_.get().get() == nullptr) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(int64_t(1000 / freq_)));
    } else {
      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

      wbcPtr_->update_trajectory_reference(refTrajPtrBuffer_.get());
      wbcPtr_->update_base_policy(vf_coeffs_buffer_.get());
      actuator_commands_ = wbcPtr_->optimize();

      publishCmds();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(
      rclcpp::get_logger("TrajectoryStabilization"), "TSC: max time %f ms,  average time %f ms",
      timer_.getMaxIntervalInMilliseconds(), timer_.getAverageInMilliseconds());
}

void TrajectoryStabilization::vector_field_loop() {
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(50.0);

  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr ||
        refTrajPtrBuffer_.get().get() == nullptr) {
      continue;
    } else {
      // RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"), "Adaptive Gain Computaion:
      // run");
      vf_construct_ptr_->update_trajectory_reference(refTrajPtrBuffer_.get());
      vf_coeffs_buffer_.push(vf_construct_ptr_->compute());
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"),
              "Vector Field Construction: max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

} // namespace clear
