#pragma once

#include <pinocchio/PinocchioInterface.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "asserts/trajectory/ReferenceBuffer.h"

using namespace rclcpp;

namespace clear {
class DataVisualization {
public:
  DataVisualization(Node::SharedPtr nodeHandle);

  ~DataVisualization();

  void update_current_state(std::shared_ptr<vector_t> qpos_ptr,
                            std::shared_ptr<vector_t> qvel_ptr);

  void set_trajectory_reference(
      std::shared_ptr<ReferenceBuffer> referenceTrajectoriesPtr);

  void update_footholds(
      std::map<std::string, std::pair<scalar_t, vector3_t>> footholds);

private:
  void inner_loop();

  void publishCurrentState();

  void publishFootholds();

  void publishBaseTrajectory();

  void publishFootTrajectory();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;

  std::string base_name, robot_name;
  std::vector<std::string> foot_names;
  std::vector<std::string> actuated_joints_name;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;
  std::shared_ptr<ReferenceBuffer> refTrajBuffer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footholds_pub_;
  Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      foot_traj_msg_publisher_;
  Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      foot_traj_ref_msg_publisher_;
  visualization_msgs::msg::MarkerArray line_strip_foot_traj_;
  visualization_msgs::msg::MarkerArray line_strip_foot_traj_ref_;
  visualization_msgs::msg::MarkerArray line_strip_footholds_;

  Publisher<visualization_msgs::msg::Marker>::SharedPtr base_traj_pub_;
  Publisher<visualization_msgs::msg::Marker>::SharedPtr base_traj_ref_pub_;
  visualization_msgs::msg::Marker line_strip_base;
  visualization_msgs::msg::Marker line_strip_base_ref;
};

} // namespace clear
