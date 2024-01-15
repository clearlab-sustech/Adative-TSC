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
#include "ocs2_legged_robot_ros/gait/GaitKeyboardPublisher.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char *argv[]) {
  const std::string robotName = "l2aurdf01";

  // Initialize ros
  rclcpp::init(argc, argv);
  auto nodeHandle =
      std::make_shared<rclcpp::Node>(robotName + "_mpc_mode_schedule");
  nodeHandle->declare_parameter("/gaitCommandFile", "");
  // Get node parameters
  std::string gaitCommandFile = nodeHandle->get_parameter("/gaitCommandFile")
                                    .get_parameter_value()
                                    .get<std::string>();
  RCLCPP_ERROR_STREAM(nodeHandle->get_logger(),
                      "Loading gait file: " << gaitCommandFile);

  GaitKeyboardPublisher gaitCommand(nodeHandle, gaitCommandFile, robotName,
                                    true);
  spin_some(nodeHandle);

  while (rclcpp::ok()) {
    gaitCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}
