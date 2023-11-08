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
#include <ocs2_legged_robot/MRT_ROS_Robot_Loop.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char **argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros
  rclcpp::init(argc, argv);
  auto nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_mrt");
  nodeHandle->declare_parameter("/referenceFile", "");
  nodeHandle->declare_parameter("/urdfFile", "");
  nodeHandle->declare_parameter("/taskFile", "");

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

  // Robot interface
  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

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

  // controller
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_ =
      std::make_shared<PinocchioEndEffectorKinematics>(
          interface.getPinocchioInterface(), pinocchioMapping,
          interface.modelSettings().contactNames3DoF);
  std::shared_ptr<WholeBodyController> wbc_ptr =
      std::make_shared<WholeBodyController>(interface.getPinocchioInterface(),
                                            interface.getCentroidalModelInfo(),
                                            *eeKinematicsPtr_);
  wbc_ptr->loadTasksSetting(taskFile, true);

  // legged robot
  auto rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo());
  MRT_ROS_Robot_Loop robot_mrt_ros(
      nodeHandle, mrt, rbdConversions_, interface.getCentroidalModelInfo(),
      wbc_ptr, interface.mpcSettings().mrtDesiredFrequency_, "ocs2_mujoco/");
  robot_mrt_ros.subscribeObservers({leggedRobotVisualizer});

  // run
  robot_mrt_ros.run();

  // Successful exit
  return 0;
}
