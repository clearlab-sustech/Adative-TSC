/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot/MRT_ROS_Robot_Loop.h"

namespace ocs2
{

  namespace legged_robot
  {
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    MRT_ROS_Robot_Loop::MRT_ROS_Robot_Loop(
        Node::SharedPtr nodeHandle, MRT_ROS_Interface &mrt,
        std::shared_ptr<CentroidalModelRbdConversions> rbdConversions,
        CentroidalModelInfo info,
        scalar_t mrtDesiredFrequency, std::string topic_prefix)
        : nodeHandle_(nodeHandle), mrt_(mrt), rbdConversions_(rbdConversions),
          info_(std::move(info)),
          mrtDesiredFrequency_(mrtDesiredFrequency)
    {
      if (mrtDesiredFrequency_ < 0)
      {
        throw std::runtime_error("MRT loop frequency should be a positive number.");
      }

      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
      actuatorCmds_pub =
          nodeHandle_->create_publisher<ocs2_msgs::msg::ActuatorCmds>(
              topic_prefix + "actuators_cmds", qos);
      joints_state_sub =
          nodeHandle_->create_subscription<sensor_msgs::msg::JointState>(
              topic_prefix + "joint_states", qos,
              std::bind(&MRT_ROS_Robot_Loop::joints_state_callback, this,
                        std::placeholders::_1));
      odom_sub = nodeHandle_->create_subscription<nav_msgs::msg::Odometry>(
          topic_prefix + "odom", qos,
          std::bind(&MRT_ROS_Robot_Loop::odom_callback, this,
                    std::placeholders::_1));
      RCLCPP_INFO(nodeHandle_->get_logger(), "MRT_ROS_Robot_Loop is ready");
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void MRT_ROS_Robot_Loop::run()
    {
      while (joints_state_ptr_.get() == nullptr || odom_ptr_.get() == nullptr)
      {
        spin_some(nodeHandle_);
      }
      vector_t rbdState = get_rbd_state();
      currentObservation.time = 0;
      currentObservation.state =
          rbdConversions_->computeCentroidalStateFromRbdModel(rbdState);
      currentObservation.input = vector_t::Zero(info_.inputDim);
      currentObservation.mode = 15; // stance
      TargetTrajectories initTargetTrajectories({0}, {currentObservation.state},
                                                {currentObservation.input});

      // Reset MPC node
      mrt_.resetMpc(initTargetTrajectories);

      RCLCPP_WARN_STREAM(nodeHandle_->get_logger(),
                         "Waiting for the initial policy ...");

      // Wait for the initial policy
      while (!mrt_.initialPolicyReceived() && rclcpp::ok())
      {
        spin_some(nodeHandle_);
        mrt_.setCurrentObservation(currentObservation);
      }
      RCLCPP_INFO_STREAM(nodeHandle_->get_logger(),
                         "Initial policy has been received.");

      start_time_ = nodeHandle_->now().seconds();
      realtime_loop();
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void MRT_ROS_Robot_Loop::realtime_loop()
    {
      rclcpp::Rate loop_rate(mrtDesiredFrequency_);
      while (rclcpp::ok())
      {
        spin_some(nodeHandle_);
        vector_t rbdState = get_rbd_state();
        currentObservation.time = nodeHandle_->now().seconds() - start_time_;
        currentObservation.state =
            rbdConversions_->computeCentroidalStateFromRbdModel(rbdState);

        /* std::cout << "### Current time " << currentObservation.time << "\n"; */

        // Update the policy if a new on was received
        mrt_.updatePolicy();

        SystemObservation nextObservation;
        mrt_.evaluatePolicy(currentObservation.time, currentObservation.state,
                            nextObservation.state, nextObservation.input,
                            nextObservation.mode);

        currentObservation.input = nextObservation.input;
        currentObservation.mode = nextObservation.mode;

        // User-defined modifications before publishing
        modifyObservation(currentObservation);

        // Publish
        mrt_.setCurrentObservation(currentObservation);
        /* std::cout << ">>> Observation is published at " <<
           currentObservation.time
                  << "\n"; */

        vector_t x;
        vector_t torque = x.tail(12);
        vector_t qpos_des =
            centroidal_model::getJointAngles(nextObservation.state, info_);
        vector_t qvel_des =
            centroidal_model::getJointVelocities(nextObservation.input, info_);

        publish_actuator_cmds(qpos_des, qvel_des, torque);

        // Update observers
        for (auto &observer : observers_)
        {
          observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
        }

        loop_rate.sleep();
      }
    }

    void MRT_ROS_Robot_Loop::publish_actuator_cmds(vector_t qpos_des,
                                                   vector_t qvel_des,
                                                   vector_t tau)
    {
      ocs2_msgs::msg::ActuatorCmds cmds;
      cmds.time_stamp = nodeHandle_->now();
      for (std::map<std::string, size_t>::iterator it = info_.joint_name2id.begin();
           it != info_.joint_name2id.end(); ++it)
      {
        cmds.actuators_name.push_back(it->first);
        size_t joint_id = it->second - 2;
        cmds.kp.push_back(30.0);
        cmds.kd.push_back(1.0);
        cmds.pos.push_back(qpos_des(joint_id));
        cmds.vel.push_back(qvel_des(joint_id));
        cmds.torque.push_back(tau(joint_id));
      }
      actuatorCmds_pub->publish(cmds);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void MRT_ROS_Robot_Loop::modifyObservation(SystemObservation &observation) {}

    vector_t MRT_ROS_Robot_Loop::get_rbd_state()
    {
      vector_t rbdState(2 * info_.generalizedCoordinatesNum);
      odom_mtx_.lock();
      joints_state_mtx_.lock();

      Eigen::Quaternion<scalar_t> quat(
          odom_ptr_->pose.pose.orientation.w, odom_ptr_->pose.pose.orientation.x,
          odom_ptr_->pose.pose.orientation.y, odom_ptr_->pose.pose.orientation.z);
      vector_t b_angVel(3);
      b_angVel << odom_ptr_->twist.twist.angular.x,
          odom_ptr_->twist.twist.angular.y, odom_ptr_->twist.twist.angular.z;
      vector_t w_angVel = quat.toRotationMatrix() * b_angVel;

      rbdState.head(3) = toEulerAnglesZYX(quat);
      rbdState.segment<3>(3) << odom_ptr_->pose.pose.position.x,
          odom_ptr_->pose.pose.position.y, odom_ptr_->pose.pose.position.z;
      rbdState.segment(info_.generalizedCoordinatesNum, 3) = w_angVel;
      rbdState.segment(info_.generalizedCoordinatesNum + 3, 3)
          << odom_ptr_->twist.twist.linear.x,
          odom_ptr_->twist.twist.linear.y, odom_ptr_->twist.twist.linear.z;

      assert(info_.actuatedDofNum == joints_state_ptr_->position.size());
      for (size_t k = 0; k < joints_state_ptr_->name.size(); k++)
      {
        size_t id = info_.joint_name2id[joints_state_ptr_->name[k]] - 2; // todo:
        /* RCLCPP_INFO(nodeHandle_->get_logger(), "joint %s: %d\n",
                    joints_state_ptr_->name[k].c_str(), id); */

        rbdState[id + 6] = joints_state_ptr_->position[k];
        rbdState[id + 6 + info_.generalizedCoordinatesNum] =
            joints_state_ptr_->velocity[k];
      }

      joints_state_mtx_.unlock();
      odom_mtx_.unlock();
      // exit(0);
      return rbdState;
    }

    void MRT_ROS_Robot_Loop::joints_state_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
      const std::lock_guard<std::mutex> lock(joints_state_mtx_);
      joints_state_ptr_ = msg;
    }

    void MRT_ROS_Robot_Loop::odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      const std::lock_guard<std::mutex> lock(odom_mtx_);
      odom_ptr_ = msg;
    }

    vector_t MRT_ROS_Robot_Loop::toEulerAnglesZYX(Eigen::Quaternion<scalar_t> q)
    {
      vector_t angles(3);
      scalar_t as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
      angles(0) =
          std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                     q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
      angles(1) = std::asin(as);
      angles(2) =
          std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                     q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
      return angles;
    }
  } // namespace legged_robot

} // namespace ocs2
