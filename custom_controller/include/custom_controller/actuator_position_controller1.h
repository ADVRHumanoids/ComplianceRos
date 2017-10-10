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

#ifndef EFFORT_CONTROLLERS__ACTUATOR_POSITION_CONTROLLER1_H
#define EFFORT_CONTROLLERS__ACTUATOR_POSITION_CONTROLLER1_H

/**
   @class effort_controllers::ActuatorPositionController1
   @brief Actuator Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "effort_controllers::ActuatorPositionController1"
   @param actuator Name of the actuator to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The actuator position to achieve.

   Publishes:

   - @b state (control_msgs::ActuatorControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/JointState.h>
#include <tinyxml.h>
#include "custom_services/updatePDGains.h"
#include "custom_services/updatePrint.h"
#include "custom_services/getJointNames.h"


namespace custom_controller
{

class ActuatorPositionController1: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  int dof;
 
  struct Commands
  {
    std::vector<double> position_; // Last commanded position
  };

  ActuatorPositionController1();
  ~ActuatorPositionController1();


  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);


  
  void setCommand(std::vector<double> pos_command);

 

  void starting(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);
  
  bool updateGains(custom_services::updatePDGains::Request &req, custom_services::updatePDGains::Response &res);
  bool updatePrint(custom_services::updatePrint::Request &req, custom_services::updatePrint::Response &res);
  bool getJointNames(custom_services::getJointNames::Request &req, custom_services::getJointNames::Response &res);

 
  void enforceJointLimits(double &command, int i);

 
  std::vector<hardware_interface::JointHandle> joint_;

  std::vector<boost::shared_ptr<const urdf::Joint> > joint_urdf_;

  realtime_tools::RealtimeBuffer<Commands> command_;

  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer

private:
  int loop_count_;
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;
  
  ros::ServiceServer update_gains_;
  ros::ServiceServer update_print_;
  ros::ServiceServer get_joint_names_;
  
  struct Actuator
  { 
      std::vector<double>  torque_limit;
  };

  struct Control
  {
      std::vector<double>  Kp;
      std::vector<double>  Kd;
      int print_data;
  };

  Actuator actuator_;
  Control controller_;

  
  void setCommandCB(const sensor_msgs::JointState  msg);

};

} // namespace

#endif
