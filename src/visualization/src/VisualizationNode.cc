#include "visualization/VisualizationNode.h"
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

using namespace rclcpp;

namespace clear {

VisualizationNode::VisualizationNode(std::string config_yaml)
    : Node("VisualizationNode") {
  auto config_ = YAML::LoadFile(config_yaml);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string estimated_state_topic =
      config_["global"]["topic_names"]["estimated_states"].as<std::string>();
  std::string mode_schedule_topic =
      config_["global"]["topic_names"]["mode_schedule"].as<std::string>();
  std::string trajectories_topic =
      config_["global"]["topic_names"]["trajectories"].as<std::string>();
  base_name = config_["model"]["base_name"].as<std::string>();
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
  estimated_state_subscription_ =
      this->create_subscription<trans::msg::EstimatedStates>(
          topic_prefix + estimated_state_topic, qos,
          std::bind(&VisualizationNode::estimated_state_callback, this,
                    std::placeholders::_1));
  mode_schedule_subscription_ =
      this->create_subscription<trans::msg::ModeScheduleTrans>(
          topic_prefix + mode_schedule_topic, qos,
          std::bind(&VisualizationNode::mode_schedule_callback, this,
                    std::placeholders::_1));
  trajectories_subscription_ =
      this->create_subscription<trans::msg::TrajectoryArray>(
          topic_prefix + trajectories_topic, qos,
          std::bind(&VisualizationNode::trajectories_callback, this,
                    std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  joint_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  foot_traj_ref_msg_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          topic_prefix + "foot_traj_ref_vis", qos);
  base_traj_ref_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      topic_prefix + "base_trajectory_ref", qos);

  line_strip_base_ref.header.frame_id = "world";
  line_strip_base_ref.action = visualization_msgs::msg::Marker::ADD;
  line_strip_base_ref.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip_base_ref.scale.x = 0.01;
  line_strip_base_ref.scale.y = 0.01;
  line_strip_base_ref.scale.z = 0.01;
  line_strip_base_ref.color.a = 0.6;
  line_strip_base_ref.color.r = 1.0;
  line_strip_base_ref.color.g = 0.0;
  line_strip_base_ref.color.b = 0.0;
  line_strip_base_ref.ns = base_name;

  line_strip_foot_traj_ref_.markers.resize(foot_names.size());
  for (size_t i = 0; i < foot_names.size(); i++) {
    line_strip_foot_traj_ref_.markers[i].header.frame_id = "world";
    line_strip_foot_traj_ref_.markers[i].action =
        visualization_msgs::msg::Marker::ADD;
    line_strip_foot_traj_ref_.markers[i].type =
        visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_foot_traj_ref_.markers[i].color.a = 0.6;
    line_strip_foot_traj_ref_.markers[i].color.r = 1.0;
    line_strip_foot_traj_ref_.markers[i].color.g = 0.0;
    line_strip_foot_traj_ref_.markers[i].color.b = 0.0;
    line_strip_foot_traj_ref_.markers[i].scale.x = 0.01;
    line_strip_foot_traj_ref_.markers[i].scale.y = 0.01;
    line_strip_foot_traj_ref_.markers[i].scale.z = 0.01;
    line_strip_foot_traj_ref_.markers[i].ns = foot_names[i];
  }

  inner_loop_thread_ = std::thread(&VisualizationNode::innerLoop, this);
  run_.push(true);
}

VisualizationNode::~VisualizationNode() {
  run_.push(false);
  inner_loop_thread_.join();
}

void VisualizationNode::estimated_state_callback(
    const trans::msg::EstimatedStates::SharedPtr msg) const {
  estimated_state_buffer.push(msg);
}

void VisualizationNode::mode_schedule_callback(
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

  // mode_schedule->print();
}

void VisualizationNode::trajectories_callback(
    const trans::msg::TrajectoryArray::SharedPtr msg) const {
  trajectories_msg_buffer_.push(msg);
}

void VisualizationNode::innerLoop() {
  rclcpp::Rate loop_rate(500);
  size_t iter = 0;
  while (rclcpp::ok() && run_.get()) {
    if (estimated_state_buffer.get().get() == nullptr ||
        mode_schedule_buffer.get().get() == nullptr ||
        trajectories_msg_buffer_.get().get() == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    publishCurrentState();

    /*--------- foot trajectory --------------*/
    if (iter % 5 == 0) {
      // RCLCPP_INFO(get_logger(), "publishFootTrajectory");
      publishFootTrajectory();
    }

    /*--------- Base Trajectory --------------*/
    if (iter % 10 == 0) {
      // RCLCPP_INFO(get_logger(), "publishBaseTrajectory");
      publishBaseTrajectory();
    }

    iter++;

    loop_rate.sleep();
  }
}

void VisualizationNode::publishBaseTrajectory() {
  trans::msg::Trajectory base_traj;
  for (auto &traj : trajectories_msg_buffer_.get()->array) {
    if (traj.header.frame_id == base_name + "_pos") {
      base_traj = traj;
      break;
    }
  }
  if (base_traj.knots.empty())
    return;

  line_strip_base_ref.header.stamp = this->now();

  line_strip_base_ref.points.clear();

  for (auto &knot : base_traj.knots) {
    geometry_msgs::msg::Point point;
    point.x = knot.x;
    point.y = knot.y;
    point.z = knot.z;
    line_strip_base_ref.points.emplace_back(point);
  }
  base_traj_ref_pub_->publish(line_strip_base_ref);
}

void VisualizationNode::publishFootTrajectory() {
  trans::msg::TrajectoryArray foot_traj;
  for (auto &traj : trajectories_msg_buffer_.get()->array) {
    for (size_t i = 0; i < foot_names.size(); i++) {
      if (traj.header.frame_id == foot_names[i]) {
        foot_traj.array.emplace_back(traj);
      }
    }
  }
  if (foot_traj.array.size() == foot_names.size()) {
    for (size_t i = 0; i < foot_names.size(); i++) {
      auto &ref_marker = line_strip_foot_traj_ref_.markers[i];

      ref_marker.header.stamp = this->now();
      ref_marker.points.clear();
      // if (ref_marker.points.size() > 200) {
      //   ref_marker.points.erase(ref_marker.points.begin());
      // }

      for (auto &knot : foot_traj.array[i].knots) {
        geometry_msgs::msg::Point point;
        point.x = knot.x;
        point.y = knot.y;
        point.z = knot.z;
        ref_marker.points.emplace_back(point);
      }
    }
    foot_traj_ref_msg_publisher_->publish(line_strip_foot_traj_ref_);
  }
}

void VisualizationNode::publishCurrentState() {
  geometry_msgs::msg::TransformStamped base_pose_msg;
  const auto &base_odom = estimated_state_buffer.get()->odom;

  base_pose_msg.header.stamp = this->now();
  base_pose_msg.header.frame_id = "world";
  base_pose_msg.child_frame_id = base_name;
  base_pose_msg.transform.translation.x = base_odom.pose.pose.position.x,
  base_pose_msg.transform.translation.y = base_odom.pose.pose.position.y,
  base_pose_msg.transform.translation.z = base_odom.pose.pose.position.z;
  base_pose_msg.transform.rotation = base_odom.pose.pose.orientation;

  tf_broadcaster_->sendTransform(base_pose_msg);
  joint_state_publisher_->publish(estimated_state_buffer.get()->joint_states);
}

} // namespace clear
