#include "AtscImpl.h"
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

  dt_ = config_["controller"]["dt"].as<scalar_t>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  estimated_state_subscription_ =
      this->create_subscription<trans::msg::EstimatedStates>(
          topic_prefix + estimated_state_topic, qos,
          std::bind(&AtscImpl::estimated_state_callback, this,
                    std::placeholders::_1));
  actuators_cmds_pub_ptr_ = this->create_publisher<trans::msg::ActuatorCmds>(
      topic_prefix + actuators_cmds_topic, qos);

  atsc_ptr = std::make_shared<AtscQuadruped>(config_yaml, this->get_logger());

  inner_loop_thread_ = std::thread(&AtscImpl::inner_loop, this);
  run_.push(true);
}

AtscImpl::~AtscImpl() {
  run_.push(false);
  inner_loop_thread_.join();
}

void AtscImpl::estimated_state_callback(
    const trans::msg::EstimatedStates::SharedPtr msg) const {
  estimated_state_buffer.push(msg);
}

void AtscImpl::inner_loop() {
  rclcpp::Rate loop_rate(1.0 / dt_);
  while (rclcpp::ok() && run_.get()) {
    if (estimated_state_buffer.get().get() == nullptr) {
      continue;
    } else {
      auto estimated_states_ptr = estimated_state_buffer.get();
      atsc_ptr->update_state(estimated_states_ptr->odom,
                             estimated_states_ptr->joint_states);

      atsc_ptr->eval();

      trans::msg::ActuatorCmds actuatorCmds;
      actuatorCmds.header.frame_id = estimated_states_ptr->odom.header.frame_id;
      actuatorCmds.header.stamp = this->now();

      auto actuator_cmds = atsc_ptr->getActuatorCmds();
      for (size_t id = 0; id < actuator_cmds->names.size(); id++) {
        actuatorCmds.names.emplace_back(actuator_cmds->names[id]);
        actuatorCmds.gain_p.emplace_back(actuator_cmds->Kp(id));
        actuatorCmds.pos_des.emplace_back(actuator_cmds->pos(id));
        actuatorCmds.gaid_d.emplace_back(actuator_cmds->Kd(id));
        actuatorCmds.vel_des.emplace_back(actuator_cmds->vel(id));
        actuatorCmds.feedforward_torque.emplace_back(actuator_cmds->tau(id));
      }

      actuators_cmds_pub_ptr_->publish(actuatorCmds);
    }
    loop_rate.sleep();
  }
}

} // namespace clear
