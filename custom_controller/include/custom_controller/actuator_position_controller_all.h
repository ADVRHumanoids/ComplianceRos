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

#ifndef EFFORT_CONTROLLERS__ACTUATOR_POSITION_CONTROLLER_ALL_H
#define EFFORT_CONTROLLERS__ACTUATOR_POSITION_CONTROLLER_ALL_H

/**
   @class custom_controller::ActuatorPositionController4
   @brief Actuator Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @PARAMETERS
   @param type: custom_controller/ActuatorPositionController4 #(required)
   @param dof: number of controlled joints #(required)
   @param print_data: #(optional)
	     - first published joint
             - second published joint
             -
             - nth published joint
   @param joint{jointnumber} (ex. joint1, joint2): #(required)
	     name: joint name in urdf #(required)
	     a_Kp: controller actuator proportional gain #(required)
	     a_Kd: controller actuator derivative gain #(required)
	     j_Kp: controller joint proportional gain #(required)
	     j_Kd: controller joint derivative gain #(required)
	     Ki: controller integral gain #(required)
	     Ks: spring stiffness #(required)
	     D: spring damping #(required)
	     Bm: motor inertia #(required)
	     phim: motor damping #(required)
	     nu: gear ration #(required)
	     Torque_limit: maximum motor torque #(required)

   Subscribes to:

   - @b command (sensor_msgs::JointState) : The actuator desired position.

   Publishes:

   - @b state (custom_messages::GroupControllerMsg) :
     Current state of the actuator and controller.

*/

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/JointState.h>
#include <tinyxml.h>
#include "custom_services/updateGains.h"
#include "custom_services/updatePrint.h"
#include "custom_services/getJointNames.h"
#include "custom_messages/GroupControllerMsg.h"
#include "custom_messages/CustomCmnd.h"
#include <std_srvs/SetBool.h>



namespace custom_controller
{

class ActuatorPositionControllerAll: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  // number of joints in controller
  int dof;
  // strcture to keep the desired positions
  struct Commands
  {
    std::vector<double> position_; // Last commanded position
    std::vector<double> velocity_; // Last commanded velocity
    std::vector<double> effort_; // Last commanded effort
    std::vector<double> onlineGain1_; // Last commanded online gain 1
    std::vector<double> onlineGain2_; // Last commanded online gain 2
  };

  ActuatorPositionControllerAll();
  ~ActuatorPositionControllerAll();


  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);


  // function to handle information outside the real-time loop
  void setCommand(std::vector<double> pos_command, std::vector<double> vel_command, std::vector<double> eff_command, std::vector<double> gain1_command, std::vector<double> gain2_command);

  // function called when the controller starts
  void starting(const ros::Time& time);

  // controller update function, called at each time step
  void update(const ros::Time& time, const ros::Duration& period);

  // controller services callbacks
  bool updateGains(custom_services::updateGains::Request &req, custom_services::updateGains::Response &res);
  bool updatePrint(custom_services::updatePrint::Request &req, custom_services::updatePrint::Response &res);
  bool getJointNames(custom_services::getJointNames::Request &req, custom_services::getJointNames::Response &res);
  bool setFeedForwardTorque(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // check if deisred position is within joint limits
  void enforceJointLimits(double &command, int i);


  std::vector<hardware_interface::JointHandle> joint_;

  std::vector<boost::shared_ptr<const urdf::Joint> > joint_urdf_;

  realtime_tools::RealtimeBuffer<Commands> command_;

  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer

private:
  // number of iterations
  int loop_count_;
  
  // flag which enables/disables feedforward torque term
  bool ff_torque_enabled = false;

  // declare publisher
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      custom_messages::GroupControllerMsg> > controller_debug_publisher_ ;
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      sensor_msgs::JointState> > controller_state_publisher_ ;
  // declare subscriber
  ros::Subscriber sub_command_;

  // declare services
  ros::ServiceServer update_gains_;
  ros::ServiceServer update_print_;
  ros::ServiceServer get_joint_names_;
  ros::ServiceServer set_ff_torque_;

  // structure to keep data related with actuators
  struct Actuator
  {
	  std::vector<double>  position;
	  std::vector<double>  velocity;
	  std::vector<double>  acceleration;
	  std::vector<double>  torque_limit;
	  std::vector<double>  motor_torque;
	  std::vector<double>  Ks;
	  std::vector<double>  D;
	  std::vector<double>  B;
	  std::vector<double>  phi;};
  // structure to keep controller data
  struct Control
  {
	  std::vector<double>  a_Kp;
	  std::vector<double>  a_Kd;
	  std::vector<double>  j_Kp;
	  std::vector<double>  j_Kd;
	  std::vector<double>  Ki;
	  std::vector<double>  integrator_error;
	  std::vector<int> print_list;
	  std::vector<int> print_data;
	  std::vector<int> joints;
	  int update;
  };


  Actuator actuator_;
  Control controller_;

  // subscriber callback function
  void setCommandCB(const custom_messages::CustomCmnd  msg);
  // service update_print internal functions
  bool addPrint(int data, std::string* response);
  bool removePrint(int data, std::string* response);


};

} // namespace


#endif
