#include "mros_hw/MrosHW.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

MrosHW::MrosHW(Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);
  robot_type_ = config_["model"]["name"].as<std::string>();

  init();
}

MrosHW::~MrosHW() {}

void MrosHW::init() {
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

 

  imuData_.ori_[0] = robotImu_.quat[1];
  imuData_.ori_[1] = robotImu_.quat[2];
  imuData_.ori_[2] = robotImu_.quat[3];
  imuData_.ori_[3] = robotImu_.quat[0];
  imuData_.angularVel_[0] = robotImu_.gyro[0];
  imuData_.angularVel_[1] = robotImu_.gyro[1];
  imuData_.angularVel_[2] = robotImu_.gyro[2];
  imuData_.linearAcc_[0] = robotImu_.acc[0];
  imuData_.linearAcc_[1] = robotImu_.acc[1];
  imuData_.linearAcc_[2] = robotImu_.acc[2];

  joints_state_ptr = std::make_shared<sensor_msgs::msg::JointState>();
  joints_state_ptr->header.frame_id = robot_type_;
  joints_state_ptr->header.stamp = nodeHandle_->now();
  joints_state_ptr->position.resize(6);
  joints_state_ptr->velocity.resize(6);
  joints_state_ptr->effort.resize(6);
  joints_state_ptr->name.resize(6);
  for (int i = 0; i < 12; ++i) {
    joints_state_ptr->position[i] = robotState_.qState[i] - jointOffsets_[i] + jointLimits_[i];
    joints_state_ptr->velocity[i] = lowState_.motorState[i].dq;
    joints_state_ptr->effort[i] = lowState_.motorState[i].tauEst;
    joints_state_ptr->name[i] = jointsIndex2NameMap[i];
    // RCLCPP_INFO(rclcpp::get_logger("MrosHW"), "low state q[%ld]=%f", i,
    //             lowState_.motorState[i].q);
  }

   for (int i = 0; i < 6; ++i) {
    jointData_[i].pos_ =
        axisAdjustList_[i] *
        (robotState_.qState[i] - jointOffsets_[i] + jointLimits_[i]);
    jointData_[i].vel_ = axisAdjustList_[i] * robotState_.qdState[i];
    jointData_[i].tau_ = axisAdjustList_[i] * robotState_.currentState[i];
  }

  imu_data_ptr = std::make_shared<sensor_msgs::msg::Imu>();
  imu_data_ptr->header.frame_id = robot_type_;
  imu_data_ptr->header.stamp = nodeHandle_->now();
  imu_data_ptr->orientation.w = lowState_.imu.quaternion[0];
  imu_data_ptr->orientation.x = lowState_.imu.quaternion[1];
  imu_data_ptr->orientation.y = lowState_.imu.quaternion[2];
  imu_data_ptr->orientation.z = lowState_.imu.quaternion[3];
  imu_data_ptr->angular_velocity.x = lowState_.imu.gyroscope[0];
  imu_data_ptr->angular_velocity.y = lowState_.imu.gyroscope[1];
  imu_data_ptr->angular_velocity.z = lowState_.imu.gyroscope[2];
  imu_data_ptr->linear_acceleration.x = lowState_.imu.accelerometer[0];
  imu_data_ptr->linear_acceleration.y = lowState_.imu.accelerometer[1];
  imu_data_ptr->linear_acceleration.z = lowState_.imu.accelerometer[2];

  touch_sensor_ptr = std::make_shared<trans::msg::TouchSensor>();
  touch_sensor_ptr->header.frame_id = robot_type_;
  touch_sensor_ptr->header.stamp = nodeHandle_->now();
  std::vector<int> foot_array = {FL_, FR_, RL_, RR_};
  std::vector<string> touch_name_array = {"FL_touch", "FR_touch", "RL_touch",
                                          "RR_touch"};
  for (size_t i = 0; i < 4; ++i) {
    touch_sensor_ptr->names.emplace_back(touch_name_array[i]);
    touch_sensor_ptr->value.emplace_back(
        static_cast<float>(lowState_.footForce[foot_array[i]]) / 20.0);
  }
}

void MrosHW::send() {
  // const auto actuator_cmd_msg = actuator_cmd_msg_buffer.get();
  // if (actuator_cmd_msg != nullptr) {
  //   const auto time_stamp =
  //       rclcpp::Time(actuator_cmd_msg->header.stamp).seconds();
  //   if (actuator_cmd_msg->header.frame_id == robot_type_ &&
  //       abs(time_stamp - nodeHandle_->now().seconds()) < 0.2) {
  //     for (size_t i = 0; i < actuator_cmd_msg->names.size(); ++i) {
  //       const auto name = actuator_cmd_msg->names[i];
  //       if (jointsName2IndexMap.find(name) != jointsName2IndexMap.end()) {
  //         const auto idx = jointsName2IndexMap[name];
  //         lowCmd_.motorCmd[idx].q =
  //             static_cast<float>(actuator_cmd_msg->pos_des[i]);
  //         lowCmd_.motorCmd[idx].dq =
  //             static_cast<float>(actuator_cmd_msg->vel_des[i]);
  //         lowCmd_.motorCmd[idx].Kp =
  //             static_cast<float>(actuator_cmd_msg->gain_p[i]);
  //         lowCmd_.motorCmd[idx].Kd =
  //             static_cast<float>(actuator_cmd_msg->gaid_d[i]);
  //         lowCmd_.motorCmd[idx].tau =
  //             static_cast<float>(actuator_cmd_msg->feedforward_torque[i]);
  //       }
  //     }
  //     safety_->PositionLimit(lowCmd_);
  //     safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  //     udp_->SetSend(lowCmd_);
  //     udp_->Send();
  //   } else if (actuator_cmd_msg->header.frame_id == robot_type_ &&
  //              abs(time_stamp - nodeHandle_->now().seconds()) >= 0.2) {
  //     for (size_t i = 0; i < 12; ++i) {
  //       lowCmd_.motorCmd[i].q = 0.0;
  //       lowCmd_.motorCmd[i].dq = 0.0;
  //       lowCmd_.motorCmd[i].Kp = 0.0;
  //       lowCmd_.motorCmd[i].Kd = 3.0;
  //       lowCmd_.motorCmd[i].tau = 0.0;
  //     }
  //     safety_->PositionLimit(lowCmd_);
  //     safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  //     udp_->SetSend(lowCmd_);
  //     udp_->Send();
  //   }

  //   // RCLCPP_FATAL(rclcpp::get_logger("MrosHW"), "send cmd msg");
  // }
  // for (size_t i = 0; i < 12; ++i) {
  //   lowCmd_.motorCmd[i].q = 0.0;
  //   lowCmd_.motorCmd[i].dq = 0.0;
  //   lowCmd_.motorCmd[i].Kp = 0.0;
  //   lowCmd_.motorCmd[i].Kd = 0.0;
  //   lowCmd_.motorCmd[i].tau = 0.0;
  // }
  // safety_->PositionLimit(lowCmd_);
  // safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  // udp_->SetSend(lowCmd_);
  // udp_->Send();
}

void MrosHW::switch_to_damping() {
  // for (size_t i = 0; i < 12; ++i) {
  //   lowCmd_.motorCmd[i].q = 0.0;
  //   lowCmd_.motorCmd[i].dq = 0.0;
  //   lowCmd_.motorCmd[i].Kp = 0.0;
  //   lowCmd_.motorCmd[i].Kd = 1.0;
  //   lowCmd_.motorCmd[i].tau = 0.0;
  // }
  // safety_->PositionLimit(lowCmd_);
  // safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  // udp_->SetSend(lowCmd_);
  // udp_->Send();
}

sensor_msgs::msg::Imu::SharedPtr MrosHW::get_imu_msg() { return imu_data_ptr; }

trans::msg::TouchSensor::SharedPtr MrosHW::get_touch_msg() {
  return touch_sensor_ptr;
}

sensor_msgs::msg::JointState::SharedPtr MrosHW::get_joint_msg() {
  return joints_state_ptr;
}

void MrosHW::set_actuator_cmds(const trans::msg::ActuatorCmds::SharedPtr msg) {
  actuator_cmd_msg_buffer.push(msg);
}

} // namespace clear
