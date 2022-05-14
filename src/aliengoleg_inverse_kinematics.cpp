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

#include "aliengoleg_inverse_kinematics.h"

#include <cmath>
#include <map>
#include "ros/ros.h"

#include <xpp_states/cartesian_declarations.h>


namespace xpp {


AlienGolegInverseKinematics::Vector3d
AlienGolegInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B, bool leftside, bool front) const
{
  double q_HAA_bf; // rear bend of knees
  double q_HFE_bf, q_KFE_bf; // forward bend of knees

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;


  // translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg
  xr = ee_pos_B;

  //Make sure we can reach the point move it closer if not
  double distanceToXr = xr.norm();
  if (distanceToXr > maxDistance) {
    ROS_INFO_STREAM(std::fixed << "TOOOOOOOOOOOOOOOOO BIIIIIIIIIIIIIIIIG");
    ROS_INFO_STREAM(std::fixed << "distanceToXR: " << distanceToXr);
    ROS_INFO_STREAM(std::fixed << "maxDistance: " << maxDistance);
    Eigen::Vector3d normalized = xr.normalized();
    ROS_INFO_STREAM(std::fixed << "OldXR: " << xr);
    ROS_INFO_STREAM(std::fixed << "Normalized: " << normalized);
    xr = normalized * maxDistance;
    ROS_INFO_STREAM(std::fixed << "NewXR: " << xr);
  }

  // xr[X] = 0.0;
  // xr[Y] = 0.083;
  // xr[Z] = -0.3884;

  // compute the HAA angle
  //ROS_INFO_STREAM(std::fixed << "XR: " << xr);
  double q1Alpha = acos(abs(xr[Y]) / sqrt(pow(xr[Z], 2) + pow(xr[Y], 2)));
  //ROS_INFO_STREAM(std::fixed << "q1alpha: " << q1Alpha);
  double q1Beta = acos(hip_offset / sqrt(pow(xr[Z], 2) + pow(xr[Y], 2)));
  //ROS_INFO_STREAM(std::fixed << "q1Beta: " << q1Beta);
  if (xr[Y] > 0) {
    q_HAA_bf = q1Alpha - q1Beta;
    //ROS_INFO_STREAM(std::fixed << "Q1 Y > 0: " << q_HAA_bf);
  } else {
    q_HAA_bf = M_PI - q1Alpha - q1Beta;
    //ROS_INFO_STREAM(std::fixed << "Q1 Y <= 0: " << q_HAA_bf);
  }
  if (leftside) { //TODO this seems more like a hack than the actual solution, Hyq does not need this
    q_HAA_bf *= -1.0;
  }
  //ROS_INFO_STREAM(std::fixed << "Q1: " << q_HAA_bf);

  // rotate into the HFE coordinate system (rot around Y)
  R << 1.0, 0.0, 0.0, 0.0, cos(q_HAA_bf), -sin(q_HAA_bf), 0.0, sin(q_HAA_bf), cos(q_HAA_bf);

  xr = (R * xr).eval();
  //ROS_INFO_STREAM(std::fixed << "Rotated XR: " << xr);

  // translate into the HFE coordinate system (along Y axis)
  xr += hfe_to_haa_y;  //distance of HFE to HAA in y direction

  
  // xr[X] = 0.0;
  // xr[Y] = 0.04;
  // xr[Z] = -0.2;

  //Calculate q2
  //ROS_INFO_STREAM(std::fixed << "Rotated/shifted XR: " << xr);
  double tmp = sqrt(pow(xr[X], 2) + pow(xr[Z], 2));
  //ROS_INFO_STREAM(std::fixed << "Hypothenuse: " << tmp);

  double psi = acos(abs(xr[X]) / tmp);
  //ROS_INFO_STREAM(std::fixed << "PSI: " << psi);
  double phi = acos(( pow(length_thigh,2) + pow(xr[X],2) + pow(xr[Z],2) - pow(length_shank,2) ) / (2*length_thigh*tmp));
  //ROS_INFO_STREAM(std::fixed << "PHI: " << phi);
  // if (xr[X] > 0) {
  //     q_HFE_bf = 0.5 * M_PI + phi - psi;
  // } else {
  //     q_HFE_bf = -0.5 * M_PI + phi + psi;
  // }
  if (front) {
    if (xr[X] < 0) {
      q_HFE_bf = 0.5 * M_PI + phi - psi;
    } else {
      q_HFE_bf = -0.5 * M_PI + phi + psi;
    }
  } else {
    if (xr[X] > 0) {
      q_HFE_bf = 0.5 * M_PI + phi - psi;
    } else {
      q_HFE_bf = -0.5 * M_PI + phi + psi;
    }
  }
  //ROS_INFO_STREAM(std::fixed << "Q2: " << q_HFE_bf);
 
  //Calculate q3
  q_KFE_bf = acos((pow(length_thigh, 2) + pow(length_shank, 2) - pow(xr[X], 2) - pow(xr[Z], 2)) / (2 * length_thigh * length_shank));
  q_KFE_bf -= M_PI;
  //ROS_INFO_STREAM(std::fixed << "Q3: " << q_KFE_bf);

  // forward knee bend
  EnforceLimits(q_HAA_bf, HAA);
  EnforceLimits(q_HFE_bf, HFE);
  EnforceLimits(q_KFE_bf, KFE);

  ROS_INFO_STREAM(std::fixed << "Q1 After Limits: " << q_HAA_bf);
  ROS_INFO_STREAM(std::fixed << "Q2 After Limits: " << q_HFE_bf);
  ROS_INFO_STREAM(std::fixed << "Q3 After Limits: " << q_KFE_bf);
  
  return Vector3d(q_HAA_bf, q_HFE_bf, q_KFE_bf);
  // return Vector3d(0.0, 0.0, 0.0);
}

void
AlienGolegInverseKinematics::EnforceLimits (double& val, HyqJointID joint) const
{
  // totally exaggerated joint angle limits
  const static double haa_min = -180;
  const static double haa_max =  90;

  const static double hfe_min = -90;
  const static double hfe_max =  90;

  const static double kfe_min = -180;
  const static double kfe_max = 0;

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;

  if (val==NAN) {
    val = 0;
    ROS_INFO_STREAM(std::fixed << "Reset NAN");
  }
}

} /* namespace xpp */