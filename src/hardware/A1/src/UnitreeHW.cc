#include "unitree_hw/UnitreeHW.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

UnitreeHW::UnitreeHW(const std::string config_yaml) : Node("UnitreeHW") {
  if (init()) {
    auto config_ = YAML::LoadFile(config_yaml);
    std::string name_prefix =
        config_["global"]["topic_prefix"].as<std::string>();
    robot_type_ = config_["model"]["name"].as<std::string>();
    RCLCPP_INFO(this->get_logger(), "robot_type: %s", robot_type_.c_str());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    std::string imu_topic =
        config_["global"]["topic_names"]["imu"].as<std::string>();
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        name_prefix + imu_topic, qos);

    std::string joints_state_topic =
        config_["global"]["topic_names"]["joints_state"].as<std::string>();
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(
            name_prefix + joints_state_topic, qos);

    std::string touch_sensor_topic =
        config_["global"]["topic_names"]["touch_sensor"].as<std::string>();
    touch_publisher_ = this->create_publisher<trans::msg::TouchSensor>(
        name_prefix + touch_sensor_topic, qos);

    scalar_t freq_imu =
        config_["simulation"]["frequency"]["imu"].as<scalar_t>();
    timers_.emplace_back(this->create_wall_timer(
        std::chrono::duration<scalar_t, std::milli>{1000.0 / freq_imu},
        std::bind(&UnitreeHW::imu_callback, this)));

    scalar_t freq_joints_state =
        config_["simulation"]["frequency"]["joints_state"].as<scalar_t>();
    timers_.emplace_back(this->create_wall_timer(
        std::chrono::duration<scalar_t, std::milli>{1000.0 / freq_joints_state},
        std::bind(&UnitreeHW::joint_callback, this)));

    scalar_t freq_touch_sensor =
        config_["simulation"]["frequency"]["touch_sensor"].as<scalar_t>();
    timers_.emplace_back(this->create_wall_timer(
        std::chrono::duration<scalar_t, std::milli>{1000.0 / freq_touch_sensor},
        std::bind(&UnitreeHW::touch_callback, this)));

    scalar_t freq_drop_old_message =
        config_["simulation"]["frequency"]["drop_old_message"].as<scalar_t>();
    timers_.emplace_back(this->create_wall_timer(
        std::chrono::duration<scalar_t, std::milli>{1000.0 /
                                                    freq_drop_old_message},
        std::bind(&UnitreeHW::drop_old_message, this)));

    std::string actuators_cmds_topic =
        config_["global"]["topic_names"]["actuators_cmds"].as<std::string>();
    actuator_cmd_subscription_ =
        this->create_subscription<trans::msg::ActuatorCmds>(
            name_prefix + actuators_cmds_topic, qos,
            std::bind(&UnitreeHW::actuator_cmd_callback, this,
                      std::placeholders::_1));
  }
}

bool UnitreeHW::init() {
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
    RCLCPP_FATAL(this->get_logger(), "Unknown robot type: %s",
                 robot_type_.c_str());
    return false;
  }
  return true;
}

void UnitreeHW::read() {
  udp_->Recv();
  udp_->GetRecv(lowState_);

  sensor_msgs::msg::JointState::SharedPtr joints_state_ptr =
      std::make_shared<sensor_msgs::msg::JointState>();
  joints_state_ptr->header.frame_id = robot_type_;
  joints_state_ptr->header.stamp = this->now();
  joints_state_ptr->position.resize(12);
  joints_state_ptr->velocity.resize(12);
  joints_state_ptr->effort.resize(12);
  joints_state_ptr->name.resize(12);
  for (int i = 0; i < 12; ++i) {
    joints_state_ptr->position[i] = lowState_.motorState[i].q;
    joints_state_ptr->velocity[i] = lowState_.motorState[i].dq;
    joints_state_ptr->effort[i] = lowState_.motorState[i].tauEst;
    joints_state_ptr->name[i] = jointsIndex2NameMap[i];
  }
  joint_state_msg_buffer.push(joints_state_ptr);

  sensor_msgs::msg::Imu::SharedPtr imu_data_ptr =
      std::make_shared<sensor_msgs::msg::Imu>();
  imu_data_ptr->header.frame_id = robot_type_;
  imu_data_ptr->header.stamp = this->now();
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
  imu_msg_buffer.push(imu_data_ptr);

  trans::msg::TouchSensor::SharedPtr touch_sensor_ptr =
      std::make_shared<trans::msg::TouchSensor>();
  touch_sensor_ptr->header.frame_id = robot_type_;
  touch_sensor_ptr->header.stamp = this->now();
  std::vector<int> foot_array = {FL_, FR_, RL_, RR_};
  std::vector<string> touch_name_array = {"FL_touch", "FR_touch", "RL_touch",
                                          "RR_touch"};
  for (size_t i = 0; i < 4; ++i) {
    touch_sensor_ptr->names.emplace_back(touch_name_array[i]);
    touch_sensor_ptr->value.emplace_back(
        static_cast<float>(lowState_.footForce[foot_array[i]]) / 20.0);
  }
  touch_msg_buffer.push(touch_sensor_ptr);
}

void UnitreeHW::write() {
  const auto actuator_cmd_msg = actuator_cmd_msg_buffer.get();
  const auto time_stamp =
      rclcpp::Time(actuator_cmd_msg->header.stamp).seconds();
  if (actuator_cmd_msg->header.frame_id == robot_type_ &&
      abs(time_stamp - this->now().seconds()) < 0.2) {
    for (int i = 0; i < actuator_cmd_msg->names.size(); ++i) {
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
  }
}

void UnitreeHW::imu_callback() {
  imu_publisher_->publish(*imu_msg_buffer.get());
}

void UnitreeHW::joint_callback() {
  joint_state_publisher_->publish(*joint_state_msg_buffer.get());
}

void UnitreeHW::touch_callback() {
  touch_publisher_->publish(*touch_msg_buffer.get());
}

void UnitreeHW::actuator_cmd_callback(
    const trans::msg::ActuatorCmds::SharedPtr msg) const {
  actuator_cmd_msg_buffer.push(msg);
}

void UnitreeHW::drop_old_message() {
  auto actuator_cmd_msg = actuator_cmd_msg_buffer.get();
  const auto time_stamp =
      rclcpp::Time(actuator_cmd_msg->header.stamp).seconds();
  if (abs(time_stamp - this->now().seconds()) > 0.2) {
    for (size_t k = 0; k < actuator_cmd_msg->names.size(); k++) {
      actuator_cmd_msg->gain_p[k] = 0.0;
      actuator_cmd_msg->pos_des[k] = 0.0;
      actuator_cmd_msg->gaid_d[k] = -3.0;
      actuator_cmd_msg->vel_des[k] = 0.0;
      actuator_cmd_msg->feedforward_torque[k] = 0.0;
    }
  }
}

} // namespace clear
