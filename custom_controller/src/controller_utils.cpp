/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Malgorzata Kamedula
 Desc: Effort position controller for the SEA with SEA simulation
*/

#include "custom_controller/controller_utils.h"
#include "custom_services/getJointNames.h"

namespace custom_controller
{

MapToUrdf::MapToUrdf(std::string topic, std::string controller)
{
  ros::Rate loop_rate(10);
  ros::NodeHandle node;

  ROS_INFO_STREAM("Waiting for callback");

  ros::Subscriber init_sub = node.subscribe<sensor_msgs::JointState>(
      topic, 1, &MapToUrdf::initCallback, this); // get urdf names

  ros::ServiceClient client =
      node.serviceClient<custom_services::getJointNames>(controller +
                                                         "/get_joint_names");
  custom_services::getJointNames srv;

  if (!node.getParam(controller + "/dof", dofs))
  {
    ROS_ERROR("Number of joints not given 'dof' (namespace: %s)",
              node.getNamespace().c_str());
    return;
  }

//  ROS_INFO_STREAM(dofs);
  id.reserve(dofs);

  while (!valid & ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // load all initialized controller names
  if (client.call(srv))
  {
    for (int i = 0; i < srv.response.joints.size(); i++)
      controller_names.push_back(srv.response.joints[i]);
  }
  else
    ROS_ERROR_STREAM("Server doesn't work");

  //	for (int i = 0; i<dofs; i++){
  //		id.push_back(-1);
  //		}
  id.resize(dofs, NON_EXISTING);
//  std::fill(id.begin(), id.end(), NON_EXISTING); // UINT_MAX -> non existing mapping

  for (int i = 0; i < controller_names.size(); i++)
  {
    for (int j = 0; j < urdf_names.size(); j++)
    {
      if (controller_names.at(i) == urdf_names.at(j))
      {
        id.at(srv.response.ids.at(i)) = j;
        //			std::cout << "We're equal, best " <<
        // srv.response.ids[i] << " & " << j << std::endl;
        break;
      }
    }
  }

 // ROS_INFO_STREAM("THE END");
}
void MapToUrdf::initCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

  for (auto& name: msg->name)
  {
    urdf_names.push_back(name);
  }

  valid = true;
}


} // namespace
