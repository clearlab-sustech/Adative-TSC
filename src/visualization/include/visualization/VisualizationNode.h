#pragma once

#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <trans/msg/estimated_states.hpp>
#include <trans/msg/mode_schedule_trans.hpp>
#include <trans/msg/trajectory_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rclcpp;

namespace clear {
class VisualizationNode : public Node {
public:
  VisualizationNode(std::string config_yaml);

  ~VisualizationNode();

private:
  void estimated_state_callback(
      const trans::msg::EstimatedStates::SharedPtr msg) const;

  void mode_schedule_callback(
      const trans::msg::ModeScheduleTrans::SharedPtr msg) const;

  void
  trajectories_callback(const trans::msg::TrajectoryArray::SharedPtr msg) const;

  void innerLoop();

  void publishCurrentState();

  void publishBaseTrajectory();

  void publishFootTrajectory();

private:
  rclcpp::Subscription<trans::msg::EstimatedStates>::SharedPtr
      estimated_state_subscription_;
  rclcpp::Subscription<trans::msg::ModeScheduleTrans>::SharedPtr
      mode_schedule_subscription_;
  rclcpp::Subscription<trans::msg::TrajectoryArray>::SharedPtr
      trajectories_subscription_;

  mutable Buffer<trans::msg::EstimatedStates::SharedPtr> estimated_state_buffer;
  mutable Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer;
  mutable Buffer<trans::msg::TrajectoryArray::SharedPtr>
      trajectories_msg_buffer_;

  std::string base_name;
  std::vector<std::string> foot_names;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      foot_traj_ref_msg_publisher_;
  visualization_msgs::msg::MarkerArray line_strip_foot_traj_ref_;

  Publisher<visualization_msgs::msg::Marker>::SharedPtr base_traj_ref_pub_;
  visualization_msgs::msg::Marker line_strip_base_ref;
};

} // namespace clear
