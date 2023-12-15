#include "unitree_hw/UnitreeHW.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

UnitreeHW::UnitreeHW(Node::SharedPtr node_handle, const std::string config_yaml)
    : node_handle_(node_handle) {
  auto config_ = YAML::LoadFile(config_yaml);
  robot_type_ = config_["model"]["name"].as<std::string>();
  init();
}

UnitreeHW::~UnitreeHW() {}

void UnitreeHW::init() {
  udp_ =
      std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  udp_->InitCmdData(lowCmd_);
  if (robot_type_ == "a1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(
        UNITREE_LEGGED_SDK::LeggedType::A1);
  } else if (robot_type_ == "aliengo") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(
        UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  } else {
    throw runtime_error("Unknown robot type: " + robot_type_);
  }
}

void UnitreeHW::read() {
  // RCLCPP_FATAL(node_handle_->get_logger(), "rec len: %d", udp_->Recv());
  udp_->Recv();
  udp_->GetRecv(lowState_);

  joints_state_ptr = std::make_shared<sensor_msgs::msg::JointState>();
  joints_state_ptr->header.frame_id = robot_type_;
  joints_state_ptr->header.stamp = node_handle_->now();
  joints_state_ptr->position.resize(12);
  joints_state_ptr->velocity.resize(12);
  joints_state_ptr->effort.resize(12);
  joints_state_ptr->name.resize(12);
  for (int i = 0; i < 12; ++i) {
    joints_state_ptr->position[i] = lowState_.motorState[i].q;
    joints_state_ptr->velocity[i] = lowState_.motorState[i].dq;
    joints_state_ptr->effort[i] = lowState_.motorState[i].tauEst;
    joints_state_ptr->name[i] = jointsIndex2NameMap[i];
    // RCLCPP_INFO(node_handle_->get_logger(), "low state q[%ld]=%f", i,
    //             lowState_.motorState[i].q);
  }

  imu_data_ptr = std::make_shared<sensor_msgs::msg::Imu>();
  imu_data_ptr->header.frame_id = robot_type_;
  imu_data_ptr->header.stamp = node_handle_->now();
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
  touch_sensor_ptr->header.stamp = node_handle_->now();
  std::vector<int> foot_array = {FL_, FR_, RL_, RR_};
  std::vector<string> touch_name_array = {"FL_touch", "FR_touch", "RL_touch",
                                          "RR_touch"};
  for (size_t i = 0; i < 4; ++i) {
    touch_sensor_ptr->names.emplace_back(touch_name_array[i]);
    touch_sensor_ptr->value.emplace_back(
        static_cast<float>(lowState_.footForce[foot_array[i]]) / 20.0);
  }
}

void UnitreeHW::send() {
  const auto actuator_cmd_msg = actuator_cmd_msg_buffer.get();
  if (actuator_cmd_msg != nullptr) {
    const auto time_stamp =
        rclcpp::Time(actuator_cmd_msg->header.stamp).seconds();
    if (actuator_cmd_msg->header.frame_id == robot_type_ &&
        abs(time_stamp - node_handle_->now().seconds()) < 0.2) {
      for (size_t i = 0; i < actuator_cmd_msg->names.size(); ++i) {
        const auto name = actuator_cmd_msg->names[i];
        if (jointsName2IndexMap.find(name) != jointsName2IndexMap.end()) {
          const auto idx = jointsName2IndexMap[name];
          lowCmd_.motorCmd[idx].q =
              static_cast<float>(actuator_cmd_msg->pos_des[i]);
          lowCmd_.motorCmd[idx].dq =
              static_cast<float>(actuator_cmd_msg->vel_des[i]);
          lowCmd_.motorCmd[idx].Kp =
              static_cast<float>(actuator_cmd_msg->gain_p[i]);
          lowCmd_.motorCmd[idx].Kd =
              static_cast<float>(actuator_cmd_msg->gaid_d[i]);
          lowCmd_.motorCmd[idx].tau =
              static_cast<float>(actuator_cmd_msg->feedforward_torque[i]);
        }
      }
      safety_->PositionLimit(lowCmd_);
      safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
      udp_->SetSend(lowCmd_);
      udp_->Send();
    } else if (actuator_cmd_msg->header.frame_id == robot_type_ &&
               abs(time_stamp - node_handle_->now().seconds()) >= 0.2) {
      for (size_t i = 0; i < 12; ++i) {
        lowCmd_.motorCmd[i].q = 0.0;
        lowCmd_.motorCmd[i].dq = 0.0;
        lowCmd_.motorCmd[i].Kp = 0.0;
        lowCmd_.motorCmd[i].Kd = 3.0;
        lowCmd_.motorCmd[i].tau = 0.0;
      }
      safety_->PositionLimit(lowCmd_);
      safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
      udp_->SetSend(lowCmd_);
      udp_->Send();
    }

    // RCLCPP_FATAL(node_handle_->get_logger(), "send cmd msg");
  }
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

void UnitreeHW::switch_to_damping() {
  for (size_t i = 0; i < 12; ++i) {
    lowCmd_.motorCmd[i].q = 0.0;
    lowCmd_.motorCmd[i].dq = 0.0;
    lowCmd_.motorCmd[i].Kp = 0.0;
    lowCmd_.motorCmd[i].Kd = 1.0;
    lowCmd_.motorCmd[i].tau = 0.0;
  }
  safety_->PositionLimit(lowCmd_);
  safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  udp_->SetSend(lowCmd_);
  udp_->Send();
}

sensor_msgs::msg::Imu::SharedPtr UnitreeHW::get_imu_msg() {
  return imu_data_ptr;
}

trans::msg::TouchSensor::SharedPtr UnitreeHW::get_touch_msg() {
  return touch_sensor_ptr;
}

sensor_msgs::msg::JointState::SharedPtr UnitreeHW::get_joint_msg() {
  return joints_state_ptr;
}

void UnitreeHW::set_actuator_cmds(
    const trans::msg::ActuatorCmds::SharedPtr msg) {
  actuator_cmd_msg_buffer.push(msg);
}

} // namespace clear
