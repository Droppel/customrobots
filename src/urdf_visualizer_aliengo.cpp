/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.
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

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include "inverse_kinematics_aliengo.h"
#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>
#include <ros/console.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "aliengo_urdf_visualizer");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  const std::string joint_desired_aliengo = "xpp/joint_aliengo_des";

  auto hyq_ik = std::make_shared<InverseKinematicsAlienGo>();
  CartesianJointConverter inv_kin_converter(hyq_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_aliengo);

  // urdf joint names
  int n_ee = 4;
  int n_j  = 3;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + 0) = "FL_hip_joint";
  joint_names.at(n_j*LF + 1) = "FL_thigh_joint";
  joint_names.at(n_j*LF + 2) = "FL_calf_joint";
  joint_names.at(n_j*RF + 0) = "FR_hip_joint";
  joint_names.at(n_j*RF + 1) = "FR_thigh_joint";
  joint_names.at(n_j*RF + 2) = "FR_calf_joint";
  joint_names.at(n_j*LH + 0) = "RL_hip_joint";
  joint_names.at(n_j*LH + 1) = "RL_thigh_joint";
  joint_names.at(n_j*LH + 2) = "RL_calf_joint";
  joint_names.at(n_j*RH + 0) = "RR_hip_joint";
  joint_names.at(n_j*RH + 1) = "RR_thigh_joint";
  joint_names.at(n_j*RH + 2) = "RR_calf_joint";

  std::string urdf = "aliengo_urdf";
  UrdfVisualizer aliengo_desired(urdf, joint_names, "base", "world",
			     joint_desired_aliengo, "aliengo_des");

  ::ros::spin();

  return 1;
}
