/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotVisualizer::LeggedRobotVisualizer(
    PinocchioInterface pinocchioInterface,
    CentroidalModelInfo centroidalModelInfo,
    const PinocchioEndEffectorKinematics &endEffectorKinematics,
    Node::SharedPtr nodeHandle, scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      centroidalModelInfo_(std::move(centroidalModelInfo)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      nodeHandle_(nodeHandle),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
  endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
  launchVisualizerNode(nodeHandle);
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::launchVisualizerNode(Node::SharedPtr nodeHandle) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_system_default);
  costDesiredBasePositionPublisher_ =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
          "/legged_robot/desiredBaseTrajectory", qos);

  costDesiredFeetPositionPublishers_.resize(
      centroidalModelInfo_.numThreeDofContacts);

  costDesiredFeetPositionPublishers_[0] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
          "/legged_robot/desiredFeetTrajectory/LF", qos);
  costDesiredFeetPositionPublishers_[1] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
          "/legged_robot/desiredFeetTrajectory/RF", qos);
  costDesiredFeetPositionPublishers_[2] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
          "/legged_robot/desiredFeetTrajectory/LH", qos);
  costDesiredFeetPositionPublishers_[3] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>(
          "/legged_robot/desiredFeetTrajectory/RH", qos);
  stateOptimizedPublisher_ =
      nodeHandle->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/legged_robot/optimizedStateTrajectory", qos);
  currentStatePublisher_ =
      nodeHandle->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/legged_robot/currentState", qos);

  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*nodeHandle);
  joint_state_publisher_ =
      nodeHandle->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                 qos);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::update(const SystemObservation &observation,
                                   const PrimalSolution &primalSolution,
                                   const CommandData &command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data,
                                 centroidal_model::getGeneralizedCoordinates(
                                     observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto timeStamp = nodeHandle_->now();
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_,
                                    primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishObservation(
    builtin_interfaces::msg::Time timeStamp,
    const SystemObservation &observation) {
  // Extract components from state
  const auto basePose =
      centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
  const auto qJoints =
      centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

  // Compute cartesian state and inputs
  const auto feetPositions =
      endEffectorKinematicsPtr_->getPosition(observation.state);
  std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    feetForces[i] = centroidal_model::getContactForces(observation.input, i,
                                                       centroidalModelInfo_);
  }

  // Publish
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode),
                          feetPositions, feetForces);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishJointTransforms(
    builtin_interfaces::msg::Time timeStamp,
    const vector_t &jointAngles) const {
  if (joint_state_publisher_.get() != nullptr) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = timeStamp;
    /* joint_state.name = {"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA",
                        "LH_HFE", "LH_KFE", "RF_HAA", "RF_HFE",
                        "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"}; */
    joint_state.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
                        
    joint_state.position = {jointAngles[0], jointAngles[1],  jointAngles[2],
                            jointAngles[3], jointAngles[4],  jointAngles[5],
                            jointAngles[6], jointAngles[7],  jointAngles[8],
                            jointAngles[9], jointAngles[10], jointAngles[11]};
    joint_state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_state.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_state_publisher_->publish(joint_state);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishBaseTransform(
    builtin_interfaces::msg::Time timeStamp, const vector_t &basePose) {
  if (joint_state_publisher_ != nullptr) {
    geometry_msgs::msg::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    baseToWorldTransform.child_frame_id = "base";

    const Eigen::Quaternion<scalar_t> q_world_base =
        getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
    baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
    baseToWorldTransform.transform.translation =
        getVectorMsg(basePose.head<3>());
    tf_broadcaster_->sendTransform(baseToWorldTransform);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishTrajectory(
    const std::vector<SystemObservation> &system_observation_array,
    scalar_t speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    scalar_t frameDuration = speed * (system_observation_array[k + 1].time -
                                      system_observation_array[k].time);
    scalar_t publishDuration = timedExecutionInSeconds([&]() {
      publishObservation(nodeHandle_->now(), system_observation_array[k]);
    });
    if (frameDuration > publishDuration) {
      rclcpp::WallRate(frameDuration - publishDuration).sleep();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishCartesianMarkers(
    builtin_interfaces::msg::Time timeStamp, const contact_flag_t &contactFlags,
    const std::vector<vector3_t> &feetPositions,
    const std::vector<vector3_t> &feetForces) const {
  // Reserve message
  const size_t numberOfCartesianMarkers = 10;
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
    markerArray.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i],
                      footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(
        getForceMarker(feetForces[i], feetPositions[i], contactFlags[i],
                       Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(
      feetForces.begin(), feetForces.end(), feetPositions.begin(),
      contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(getSupportPolygonMarker(
      feetPositions.begin(), feetPositions.end(), contactFlags.begin(),
      Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
               getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_->publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishDesiredTrajectory(
    builtin_interfaces::msg::Time timeStamp,
    const TargetTrajectories &targetTrajectories) {
  const auto &stateTrajectory = targetTrajectories.stateTrajectory;
  const auto &inputTrajectory = targetTrajectories.inputTrajectory;

  // Reserve com messages
  std::vector<geometry_msgs::msg::Point> desiredBasePositionMsg;
  desiredBasePositionMsg.reserve(stateTrajectory.size());

  // Reserve feet messages
  feet_array_t<std::vector<geometry_msgs::msg::Point>> desiredFeetPositionMsgs;
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
  }

  for (size_t j = 0; j < stateTrajectory.size(); j++) {
    const auto state = stateTrajectory.at(j);
    vector_t input(centroidalModelInfo_.inputDim);
    if (j < inputTrajectory.size()) {
      input = inputTrajectory.at(j);
    } else {
      input.setZero();
    }

    // Construct base pose msg
    const auto basePose =
        centroidal_model::getBasePose(state, centroidalModelInfo_);
    geometry_msgs::msg::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());

    // Fill message containers
    desiredBasePositionMsg.push_back(pose.position);

    // Fill feet msgs
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data,
                                 centroidal_model::getGeneralizedCoordinates(
                                     state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      geometry_msgs::msg::Pose footPose;
      footPose.position = getPointMsg(feetPositions[i]);
      desiredFeetPositionMsgs[i].push_back(footPose.position);
    }
  }

  // Headers
  auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green,
                               trajectoryLineWidth_);
  comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
  comLineMsg.id = 0;

  // Publish
  costDesiredBasePositionPublisher_->publish(comLineMsg);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]),
                                  feetColorMap_[i], trajectoryLineWidth_);
    footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    footLineMsg.id = 0;
    costDesiredFeetPositionPublishers_[i]->publish(footLineMsg);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishOptimizedStateTrajectory(
    builtin_interfaces::msg::Time timeStamp,
    const scalar_array_t &mpcTimeTrajectory,
    const vector_array_t &mpcStateTrajectory,
    const ModeSchedule &modeSchedule) {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    return; // Nothing to publish
  }

  // Reserve Feet msg
  feet_array_t<std::vector<geometry_msgs::msg::Point>> feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(),
                [&](std::vector<geometry_msgs::msg::Point> &v) {
                  v.reserve(mpcStateTrajectory.size());
                });

  // Reserve Com Msg
  std::vector<geometry_msgs::msg::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(
      mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
      [&](const vector_t &state) {
        const auto basePose =
            centroidal_model::getBasePose(state, centroidalModelInfo_);

        // Fill com position and pose msgs
        geometry_msgs::msg::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());
        mpcComPositionMsgs.push_back(pose.position);

        // Fill feet msgs
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(
            model, data,
            centroidal_model::getGeneralizedCoordinates(state,
                                                        centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto feetPositions =
            endEffectorKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
          const auto position = getPointMsg(feetPositions[i]);
          feetMsgs[i].push_back(position);
        }
      });

  // Convert feet msgs to Array message
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts +
                              2); // 1 trajectory per foot + 1 for the future
                                  // footholds + 1 for the com trajectory
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    markerArray.markers.emplace_back(getLineMsg(
        std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(
      std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::msg::Marker sphereList;
  sphereList.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  const auto &eventTimes = modeSchedule.eventTimes;
  const auto &subsystemSequence = modeSchedule.modeSequence;
  const auto tStart = mpcTimeTrajectory.front();
  const auto tEnd = mpcTimeTrajectory.back();
  for (size_t event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] &&
        eventTimes[event] < tEnd) { // Only publish future footholds within the
                                    // optimized horizon
      const auto preEventContactFlags =
          modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags =
          modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const auto postEventState = LinearInterpolation::interpolate(
          eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

      const auto &model = pinocchioInterface_.getModel();
      auto &data = pinocchioInterface_.getData();
      pinocchio::forwardKinematics(model, data,
                                   centroidal_model::getGeneralizedCoordinates(
                                       postEventState, centroidalModelInfo_));
      pinocchio::updateFramePlacements(model, data);

      const auto feetPosition =
          endEffectorKinematicsPtr_->getPosition(postEventState);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        if (!preEventContactFlags[i] &&
            postEventContactFlags[i]) { // If a foot lands, a marker is added at
                                        // that location.
          sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
               getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisher_->publish(markerArray);
}

} // namespace legged_robot
} // namespace ocs2
