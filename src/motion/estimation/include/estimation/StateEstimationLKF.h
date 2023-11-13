#pragma once
#include <core/misc/Buffer.h>
#include <core/types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trans/msg/estimated_states.hpp>
#include <trans/msg/touch_sensor.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

namespace clear {
class StateEstimationLKF {
public:
  StateEstimationLKF(Node::SharedPtr nodeHandle, std::string config_yaml);

  ~StateEstimationLKF();

  std::shared_ptr<vector_t> getQpos();

  std::shared_ptr<vector_t> getQvel();

  void setContactFlag(vector<bool> flag);

  void set_imu_msg(sensor_msgs::msg::Imu::SharedPtr msg);

  void set_touch_msg(trans::msg::TouchSensor::SharedPtr msg);

  void set_joint_msg(sensor_msgs::msg::JointState::SharedPtr msg);

private:
  void setup();

  void angularMotionEstimate(const sensor_msgs::msg::Imu &imu_data,
                             std::shared_ptr<vector_t> qpos,
                             std::shared_ptr<vector_t> qvel);

  void linearMotionEstimate(const sensor_msgs::msg::Imu &imu_data,
                            std::shared_ptr<vector_t> qpos,
                            std::shared_ptr<vector_t> qvel);

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const;

  void touch_callback(const trans::msg::TouchSensor::SharedPtr msg) const;

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  void inner_loop();

private:
  Node::SharedPtr nodeHandle_;
  std::unique_ptr<PinocchioInterface> pinocchioInterface_ptr;
  std::string robot_name;
  std::vector<string> foot_names;
  vector<bool> cflag_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;
  mutable Buffer<sensor_msgs::msg::Imu::SharedPtr> imu_msg_buffer;
  mutable Buffer<trans::msg::TouchSensor::SharedPtr> touch_msg_buffer;
  mutable Buffer<sensor_msgs::msg::JointState::SharedPtr>
      joint_state_msg_buffer;
  mutable Buffer<nav_msgs::msg::Odometry::SharedPtr> odom_msg_buffer;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<trans::msg::TouchSensor>::SharedPtr touch_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joints_state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  bool use_odom_ = false;

  scalar_t dt_;
  Eigen::Matrix<scalar_t, 12, 1> ps_;
  Eigen::Matrix<scalar_t, 12, 1> vs_;
  Eigen::Matrix<scalar_t, 18, 18> A_;
  Eigen::Matrix<scalar_t, 18, 3> B_;
  Eigen::Matrix<scalar_t, 28, 18> C_;
  Eigen::Matrix<scalar_t, 18, 18> Sigma_;
  Eigen::Matrix<scalar_t, 18, 18> Q0_;
  Eigen::Matrix<scalar_t, 28, 28> R0_;
  Eigen::Matrix<scalar_t, 18, 1> x_est;
};
} // namespace clear