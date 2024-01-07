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

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char **argv) {
  const std::string robotName = "l2aurdf01";

  // Initialize ros
  rclcpp::init(argc, argv);
  auto nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_mrt");
  nodeHandle->declare_parameter(
      "/referenceFile", "/home/poplar/Desktop/VF-TSC/src/third_parties/ocs2/"
                        "ocs2_legged_robot/config/command/reference.info");
  nodeHandle->declare_parameter(
      "/urdfFile", "/home/poplar/Desktop/VF-TSC/src/asserts/a1/a1_0.urdf");
  nodeHandle->declare_parameter("/taskFile",
                                "/home/poplar/Desktop/VF-TSC/src/third_parties/"
                                "ocs2/ocs2_legged_robot/config/mpc/task.info");

  // Get node parameters
  std::string referenceFile = nodeHandle->get_parameter("/referenceFile")
                                  .get_parameter_value()
                                  .get<std::string>();
  std::string urdfFile = nodeHandle->get_parameter("/urdfFile")
                             .get_parameter_value()
                             .get<std::string>();
  std::string taskFile = nodeHandle->get_parameter("/taskFile")
                             .get_parameter_value()
                             .get<std::string>();
  RCLCPP_INFO(nodeHandle->get_logger(), "LeggedRobotInterface start");

  // Robot interface
  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

  RCLCPP_INFO(nodeHandle->get_logger(), "LeggedRobotInterface done");

  // MRT
  MRT_ROS_Interface mrt(nodeHandle, robotName);
  mrt.initRollout(&interface.getRollout());

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(
      interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(
      interface.getPinocchioInterface(), pinocchioMapping,
      interface.modelSettings().contactNames3DoF);
  auto leggedRobotVisualizer = std::make_shared<LeggedRobotVisualizer>(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
      endEffectorKinematics, nodeHandle);

  // Dummy legged robot
  MRT_ROS_Dummy_Loop leggedRobotDummySimulator(
      nodeHandle, mrt, interface.mpcSettings().mrtDesiredFrequency_,
      interface.mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummySimulator.subscribeObservers({leggedRobotVisualizer});

  // Initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input =
      vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  initObservation.mode = ModeNumber::STANCE;
  initObservation.time = 5.0;

  // Initial command
  TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state},
                                            {initObservation.input});

  // run dummy
  leggedRobotDummySimulator.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
