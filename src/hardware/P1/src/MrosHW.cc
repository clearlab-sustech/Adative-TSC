#include "mros_hw/MrosHW.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

MrosHW::MrosHW(Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle) {
  // Initialization data
  atomicRobotCmd_ = new mros::controller_msgs::RobotCmdPointFoot();
  atomicRobotState_ = new mros::controller_msgs::RobotStatePointFoot();
  atomicRobotImu_ = new mros::controller_msgs::IMUData();

  // Publisher is used to publish robot control data, topic name:
  // RobotCmdPointFoot
  robotCmdPub_ = new mros::Publisher(
      "RobotCmdPointFoot", mros::controller_msgs::RobotCmdPointFoot());
  mros::nh()->advertise(*robotCmdPub_);

  // Subscribe to state data, topic name: RobotStatePointFoot
  robotStateSub_ =
      new mros::Subscriber<mros::controller_msgs::RobotStatePointFoot>(
          "RobotStatePointFoot",
          [this](
              const mros::controller_msgs::RobotStatePointFootConstPtr &msg) {
            *atomicRobotState_ = *msg;
            atomicRobotStateOn_ = 1;
            std::this_thread::yield();
          });
  mros::nh()->subscribe(*robotStateSub_);

  // Subscribe to imu data, topic name: ImuData
  robotImuSub_ = new mros::Subscriber<mros::controller_msgs::IMUData>(
      "ImuData", [this](const mros::controller_msgs::IMUDataConstPtr &msg) {
        *atomicRobotImu_ = *msg;
        atomicRobotImuOn_ = 1;
        std::this_thread::yield();
      });
  mros::nh()->subscribe(*robotImuSub_);

  init();
}

MrosHW::~MrosHW() {}

void MrosHW::init() {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);
  robot_type_ = config_["model"]["name"].as<std::string>();

  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();

  for (const auto &joint : actuated_joints_name) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.find("_L_") != std::string::npos) {
      leg_index = 0;
    } else if (joint.find("_R_") != std::string::npos) {
      leg_index = 1;
    } else {
      continue;
    }

    if (joint.find("abad") != std::string::npos) {
      joint_index = 0;
    } else if (joint.find("hip") != std::string::npos) {
      joint_index = 1;
    } else if (joint.find("knee") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;

    auto joint_limit_settings =
        config_["model"]["joint_limit"].as<std::map<std::string, scalar_t>>();
    auto joint_offset_settings =
        config_["model"]["joint_offset"].as<std::map<std::string, scalar_t>>();

    jointLimits_[index] = joint_limit_settings[joint];
    jointOffsets_[index] = joint_offset_settings[joint];

    jointsName2IndexMap[joint] = index;

    std::cerr << "joint limit: " << joint << "-->" << jointLimits_[index]
              << std::endl;
    std::cerr << "joint offset: " << joint << "-->" << jointOffsets_[index]
              << std::endl;
  }
}

void MrosHW::read() {
  // RCLCPP_FATAL(rclcpp::get_logger("MrosHW"), "rec len: %d", udp_->Recv());
  if (atomicRobotImuOn_ == 1) {
    robotImu_ = *atomicRobotImu_;
    atomicRobotImuOn_ = 0;
  }
  if (atomicRobotStateOn_ == 1) {
    robotState_ = *atomicRobotState_;
    atomicRobotStateOn_ = 0;
  }

  joints_state_ptr = std::make_shared<sensor_msgs::msg::JointState>();
  joints_state_ptr->header.frame_id = robot_type_;
  joints_state_ptr->header.stamp = nodeHandle_->now();
  joints_state_ptr->position.resize(6);
  joints_state_ptr->velocity.resize(6);
  joints_state_ptr->effort.resize(6);
  joints_state_ptr->name.resize(6);
  for (int i = 0; i < 6; ++i) {
    joints_state_ptr->position[i] =
        robotState_.qState[i] - jointOffsets_[i] + jointLimits_[i];
    joints_state_ptr->velocity[i] = robotState_.qdState[i];
    joints_state_ptr->effort[i] = robotState_.currentState[i];
    joints_state_ptr->name[i] = actuated_joints_name[i];
  }

  imu_data_ptr = std::make_shared<sensor_msgs::msg::Imu>();
  imu_data_ptr->header.frame_id = robot_type_;
  imu_data_ptr->header.stamp = nodeHandle_->now();
  imu_data_ptr->orientation.w = robotImu_.quat[0];
  imu_data_ptr->orientation.x = robotImu_.quat[1];
  imu_data_ptr->orientation.y = robotImu_.quat[2];
  imu_data_ptr->orientation.z = robotImu_.quat[3];
  imu_data_ptr->angular_velocity.x = robotImu_.gyro[0];
  imu_data_ptr->angular_velocity.y = robotImu_.gyro[1];
  imu_data_ptr->angular_velocity.z = robotImu_.gyro[2];
  imu_data_ptr->linear_acceleration.x = robotImu_.acc[0];
  imu_data_ptr->linear_acceleration.y = robotImu_.acc[1];
  imu_data_ptr->linear_acceleration.z = robotImu_.acc[2];
}

void MrosHW::send() {
  const auto actuator_cmd_msg = actuator_cmd_msg_buffer.get();
  if (actuator_cmd_msg != nullptr) {
    const auto time_stamp =
        rclcpp::Time(actuator_cmd_msg->header.stamp).seconds();
    if (actuator_cmd_msg->header.frame_id == robot_type_ &&
        abs(time_stamp - nodeHandle_->now().seconds()) < 0.2) {
      for (size_t i = 0; i < actuator_cmd_msg->names.size(); ++i) {
        const auto name = actuator_cmd_msg->names[i];
        if (jointsName2IndexMap.find(name) != jointsName2IndexMap.end()) {
          const auto idx = jointsName2IndexMap[name];
          robotCmd_.qCmd[idx] =
              static_cast<float>(actuator_cmd_msg->pos_des[i] +
                                 jointOffsets_[i] - jointLimits_[i]);
          robotCmd_.qdCmd[idx] =
              static_cast<float>(actuator_cmd_msg->vel_des[i]);
          robotCmd_.kpCmd[idx] =
              static_cast<float>(actuator_cmd_msg->gain_p[i]);
          robotCmd_.kdCmd[idx] =
              static_cast<float>(actuator_cmd_msg->gaid_d[i]);
          robotCmd_.currentCmd[idx] =
              static_cast<float>(actuator_cmd_msg->feedforward_torque[i]);
          robotCmd_.mode[idx] = 2;
        }
      }
    } else if (actuator_cmd_msg->header.frame_id == robot_type_ &&
               abs(time_stamp - nodeHandle_->now().seconds()) >= 0.2) {
      for (size_t i = 0; i < 6; ++i) {
        robotCmd_.qCmd[i] = 0.0;
        robotCmd_.qdCmd[i] = 0.0;
        robotCmd_.kpCmd[i] = 0.0;
        robotCmd_.kdCmd[i] = 1.5;
        robotCmd_.currentCmd[i] = 0.0;
        robotCmd_.mode[i] = 0;
      }
    }
    robotCmdPub_->publish(robotCmd_);
    // RCLCPP_FATAL(rclcpp::get_logger("MrosHW"), "send cmd msg");
  }
}

void MrosHW::switch_to_damping() {
  for (size_t i = 0; i < 6; ++i) {
    robotCmd_.qCmd[i] = 0.0;
    robotCmd_.qdCmd[i] = 0.0;
    robotCmd_.kpCmd[i] = 0.0;
    robotCmd_.kdCmd[i] = 1.5;
    robotCmd_.currentCmd[i] = 0.0;
    robotCmd_.mode[i] = 0;
  }
  robotCmdPub_->publish(robotCmd_);
}

sensor_msgs::msg::Imu::SharedPtr MrosHW::get_imu_msg() { return imu_data_ptr; }

sensor_msgs::msg::JointState::SharedPtr MrosHW::get_joint_msg() {
  return joints_state_ptr;
}

void MrosHW::set_actuator_cmds(const trans::msg::ActuatorCmds::SharedPtr msg) {
  actuator_cmd_msg_buffer.push(msg);
}

} // namespace clear
