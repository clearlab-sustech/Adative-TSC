/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/CentroidalModelRbdConversions.h"
#include "ocs2_msgs/msg/actuator_cmds.hpp"
#include "ocs2_ros_interfaces/mrt/DummyObserver.h"
#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

namespace ocs2 {
namespace legged_robot {

/**
 * This class implements a loop to test MPC-MRT communication interface using
 * ROS.
 */
class MRT_ROS_Robot_Loop {
public:
  /**
   * Constructor.
   *
   * @param [in] mrt: The underlying MRT class to be used. If MRT contains a
   * rollout object, the dummy will roll out the received controller using the
   * MRT::rolloutPolicy() method instead of just sending back a planned state.
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should
   * always set to a positive number.
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a
   * positive number, MPC loop will be simulated to run by this frequency. Note
   * that this might not be the MPC's real-time frequency.
   */
  MRT_ROS_Robot_Loop(
      Node::SharedPtr nodeHandle, MRT_ROS_Interface &mrt,
      std::shared_ptr<CentroidalModelRbdConversions> rbdConversions,
      CentroidalModelInfo info,
      scalar_t mrtDesiredFrequency, std::string topic_prefix = "robot/");

  /**
   * Destructor.
   */
  virtual ~MRT_ROS_Robot_Loop() = default;

  /**
   * Runs the dummy MRT loop.
   *
   * @param [in] initObservation: The initial observation.
   * @param [in] initTargetTrajectories: The initial TargetTrajectories.
   */
  void run();

  /**
   * Subscribe a set of observers to the dummy loop. Observers are updated in
   * the provided order at the end of each timestep. The previous list of
   * observers is overwritten.
   *
   * @param observers : vector of observers.
   */
  void subscribeObservers(
      const std::vector<std::shared_ptr<DummyObserver>> &observers) {
    observers_ = observers;
  }

protected:
  /**
   * A user-defined function which modifies the observation before publishing.
   *
   * @param [in] observation: The current observation.
   */
  virtual void modifyObservation(SystemObservation &observation);

private:
  void realtime_loop();

  void publish_actuator_cmds(vector_t qpos_des, vector_t qvel_des,
                             vector_t tau);

  vector_t get_rbd_state();

  void joints_state_callback(
      const sensor_msgs::msg::JointState::SharedPtr msg) const;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  vector_t toEulerAnglesZYX(Eigen::Quaternion<scalar_t> q);

  Node::SharedPtr nodeHandle_;
  MRT_ROS_Interface &mrt_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
  CentroidalModelInfo info_;
  SystemObservation currentObservation;
  scalar_t start_time_ = 0.0;

  std::vector<std::shared_ptr<DummyObserver>> observers_;

  scalar_t mrtDesiredFrequency_;

  Publisher<ocs2_msgs::msg::ActuatorCmds>::SharedPtr actuatorCmds_pub;
  Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_state_sub;
  mutable std::mutex joints_state_mtx_;
  mutable sensor_msgs::msg::JointState::SharedPtr joints_state_ptr_;
  Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  mutable std::mutex odom_mtx_;
  mutable nav_msgs::msg::Odometry::SharedPtr odom_ptr_;
};
} // namespace legged_robot

} // namespace ocs2
