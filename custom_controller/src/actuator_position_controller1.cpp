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

#include "custom_controller/actuator_position_controller1.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <math.h>

namespace custom_controller {

ActuatorPositionController1::ActuatorPositionController1()
  : loop_count_(0)
{}

ActuatorPositionController1::~ActuatorPositionController1()
{
  sub_command_.shutdown();
}

bool ActuatorPositionController1::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();}

  if (!n.getParam("dof", dof))
  {
    ROS_ERROR("Not number of joints given 'dof' (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

    actuator_.torque_limit.reserve(dof);
    controller_.Kp.reserve(dof);
    controller_.Kd.reserve(dof);
    joint_.reserve(dof);

  for (int i=0; i<dof; i++){

  bool verify[4] = {false, false, false, false};

  std::stringstream j;
  j << "joint" << i+1 ;

  XmlRpc::XmlRpcValue joint_list;
  if (!n.getParam(j.str(), joint_list))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  if(joint_list.getType() == XmlRpc::XmlRpcValue::TypeStruct){
      double nu;
      for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = joint_list.begin(); it != joint_list.end(); ++it)   {
        std::string value = (std::string)(it->first) ;
        if( value == "Kp"){

            if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
                int Kp = joint_list[it->first];
                controller_.Kp[i] = Kp;
                }
            else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                controller_.Kp[i] = joint_list[it->first];
            else{
                ROS_ERROR_STREAM("Wrong parameter type: 'Kp' - excpected Integer or Double");
                return false;
            }
            verify[0] = true;
            }
       else if( value == "Kd"){

            if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
                int Kd = joint_list[it->first];
                controller_.Kd[i] = Kd;
                }
            else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                controller_.Kd[i] = joint_list[it->first];
            else{
                ROS_ERROR_STREAM("Wrong parameter type: 'Kd' - excpected Integer or Double");
                return false;
                }
                verify[1] = true;
        }
            else if( value == "Torque_limit"){

                if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
					int torque_limit = joint_list[it->first];
					actuator_.torque_limit[i] = torque_limit;

                }
				else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
					actuator_.torque_limit[i] = joint_list[it->first];
				else{
					ROS_ERROR_STREAM("Wrong parameter type: 'Torque_limit' - excpected Integer or Double");
					return false;
                }

            verify[2] = true;
        }
            else if ( value == "name"){
                if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeString){

                    std::string joint_name = joint_list[it->first];
                    joint_.push_back(robot->getHandle(joint_name));
                    // Get URDF info about joint
                    urdf::Model urdf;

                    if (!urdf.initParam("robot_description")){
                        ROS_ERROR("Failed to parse urdf file");
                        return false;
                    }
                    joint_urdf_.push_back(urdf.getJoint(joint_name));

                    if (!joint_urdf_[i]){
                        ROS_ERROR("Could not find joint in urdf");
                        return false;
                    }
                }
                else{
                    ROS_ERROR_STREAM("Wrong parameter type: 'Name' - excpected String");
                    return false;
                }
                verify[3] = true;
                }
            else ROS_WARN_STREAM("unknow parameter: '" << value << "' in definition of " <<  j.str());
  }
    if(!verify[0]) {ROS_ERROR_STREAM("Excpected parameter 'Kp' not defined in " << j.str()); return false;}
    if(!verify[1]) {ROS_ERROR_STREAM("Excpected parameter 'Kd' not defined in " << j.str()); return false;}
    if(!verify[2]) {ROS_ERROR_STREAM("Excpected parameter 'Torque_limit' not defined in " << j.str()); return false;}
    if(!verify[3]) {ROS_ERROR_STREAM("Excpected parameter 'name' not defined in " << j.str()); return false;}
  }
  else
    ROS_ERROR_STREAM("Wrong .yaml structure");
  }
/*
    ROS_DEBUG_STREAM("Ks1:" << actuator_.Ks[0] << " Ks2:" << actuator_.Ks[1]);
    ROS_DEBUG_STREAM("B1:" << actuator_.B[0] << "B2:" << actuator_.B[1]);
    ROS_DEBUG_STREAM("Kp1:" << controller_.Kp[0] << " Kp2:" << controller_.Kp[1]);
    ROS_DEBUG_STREAM("D1:" << actuator_.D[0] << "B2:" << actuator_.D[1]);
    ROS_DEBUG_STREAM("Kd1:" << controller_.Kd[0] << " Kd2:" << controller_.Kd[1]);
    ROS_DEBUG_STREAM("phi1:" << actuator_.phi[0] << "phi2:" << actuator_.phi[1]);
    ROS_DEBUG_STREAM("Ki1:" << controller_.Ki[0] << " Ki2:" << controller_.Ki[1]);
    ROS_DEBUG_STREAM("Torque_limit1:" << actuator_.torque_limit[0] << "Torque_limit2:" << actuator_.torque_limit[1]);
*/
 /*
  // Get actuator handle from hardware interface
  joint_[i] = robot->getHandle(joint_name[i]);
  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_[i] = urdf.getJoint(joint_name[i]);
  if (!joint_urdf_[i])
  {
    ROS_ERROR("Could not find joint in urdf");
    return false;
  }
*/
  // Check which joint data to print
  if (!n.getParam("print_data", controller_.print_data))
  {
    ROS_WARN("None printing joint specify in: %s", n.getNamespace().c_str());
    controller_.print_data = -1;
  }
  else if (controller_.print_data < 0 || controller_.print_data >= dof) {
    ROS_WARN_STREAM("Received joint number: " <<  controller_.print_data << " is not defined within the controller: " << n.getNamespace().c_str());
    controller_.print_data = -1;
  }
  else
    ROS_INFO_STREAM("Print information for a " <<  controller_.print_data << " joint of the controller: " << n.getNamespace().c_str());

   // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<sensor_msgs::JointState >("command", 1, &ActuatorPositionController1::setCommandCB, this);

  update_gains_ = n.advertiseService("update_gains", &ActuatorPositionController1::updateGains, this);
  update_print_ = n.advertiseService("update_print", &ActuatorPositionController1::updatePrint, this);
  get_joint_names_ = n.advertiseService("get_joint_names", &ActuatorPositionController1::getJointNames, this);

  return true;
}



// Set the actuator position command with a velocity command as well
void ActuatorPositionController1::setCommand(std::vector<double> pos_command)
{
  command_struct_.position_.clear();
  command_struct_.position_ = pos_command;

  command_.writeFromNonRT(command_struct_);
}

void ActuatorPositionController1::starting(const ros::Time& time)
{
  command_struct_.position_.reserve(dof);
  for (int i=0; i<dof; i++)
    command_struct_.position_.push_back(0);

  command_.initRT(command_struct_);
}

bool ActuatorPositionController1::updateGains(custom_services::updatePDGains::Request &req, custom_services::updatePDGains::Response &res){
	
	if (req.nr > -1 & req.nr < dof){
		std::stringstream msg;
		res.success = true;
		if (req.p >= 0)
			controller_.Kp[req.nr] = req.p;
		else{
			msg << "Given proportional gain is negative. Please give a positive number." << std::endl << "Proportional gain not changed." << std::endl;
			res.success = false;
			}
		
		if (req.d >= 0)
			controller_.Kd[req.nr] = req.d;
		else{
			msg << "Given derivative gain is negative. Please give a positive number." << std::endl << "Derivative gain not changed." << std::endl;
			res.success = false;
			}
	
		
		msg << "Changed controller gains for" << std::endl << "joint number: " << req.nr << std:: endl << "joint name: " << joint_[req.nr].getName() << "." << std::endl << "Current gains are: " << std::endl << "Kp: " << controller_.Kp[req.nr] << std::endl << "Kd: " << controller_.Kd[req.nr] << std::endl;
		res.message = msg.str();	
		return true;}
	else{
		res.success = false;
		std::stringstream msg;
		msg  << "Received number: " <<  req.nr << " is not defined within the controller. Please send a number bettween 0 and " << dof-1 << "." << std::endl << "To see loaded joints call 'get_joint_names' service.";
		res.message = msg.str();		
		return true;}
	}
	
bool ActuatorPositionController1::updatePrint(custom_services::updatePrint::Request &req, custom_services::updatePrint::Response &res){
	if (req.nr > -1 & req.nr < dof){
		controller_.print_data = req.nr;
		res.success = true;
		std::stringstream msg;
		msg << "Succesfully changed printed information:" << std:: endl << "current joint number: " << req.nr << std:: endl <<  "current joint name: " << joint_[req.nr].getName();
		res.message = msg.str();		
		return true;}
	else{
		res.success = false;
		std::stringstream msg;
		msg  << "Received number: " <<  req.nr << " is not defined within the controller. Please send a number bettween 0 and " << dof-1 << "." << std::endl << "To see loaded joints call 'get_joint_names' service.";
		res.message = msg.str();		
		return true;}
	}
	
bool ActuatorPositionController1::getJointNames(custom_services::getJointNames::Request &req, custom_services::getJointNames::Response &res){
	
	res.success = true;
	std::stringstream msg;
	msg << "joint number: \t joint name" << std::endl; 

	for (int j=0; j<dof; j++){
		msg << j <<  "\t" << joint_[j].getName() << std::endl; 
	}
	
	res.message = msg.str();		

	return true;
	}

void ActuatorPositionController1::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());
  double commanded_effort[dof];
  double error;

  for (int i=0; i<dof; i++){
  //commanded_effort[i] = 0;

  double command_position = command_struct_.position_[i];

/////////////////////////////////////////////////

    enforceJointLimits(command_position, i);
    error =  command_position - joint_[i].getPosition();

    commanded_effort[i] = controller_.Kp[i]*error - controller_.Kd[i]*joint_[i].getVelocity();
    commanded_effort[i] = (fabs(commanded_effort[i]) < actuator_.torque_limit[i]) ? commanded_effort[i] : commanded_effort[i]/fabs(commanded_effort[i])*actuator_.torque_limit[i];

    if(i == controller_.print_data && controller_state_publisher_ && controller_state_publisher_->trylock()){
      controller_state_publisher_->msg_.header.stamp =  time;
      controller_state_publisher_->msg_.set_point = command_position;
      controller_state_publisher_->msg_.process_value =  joint_[i].getPosition();
      controller_state_publisher_->msg_.process_value_dot =  joint_[i].getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec() ;
      controller_state_publisher_->msg_.command = commanded_effort[i];

      controller_state_publisher_->unlockAndPublish();

    }

  for (int i=0; i<dof; i++){
    joint_[i].setCommand(commanded_effort[i]);
}
    }
}

void ActuatorPositionController1::setCommandCB(const sensor_msgs::JointState  msg)
{
  setCommand(msg.position);
}

void ActuatorPositionController1::enforceJointLimits(double &command, int i)
{
  // Check that this joint has applicable limits
  if (joint_urdf_[i]->type == urdf::Joint::REVOLUTE)
  {
    if( command > joint_urdf_[i]->limits->upper ) // above upper limnit
    {
      command = joint_urdf_[i]->limits->upper;
 //     ROS_WARN_STREAM("reached upper limit: " << joint_[i].getName());
    }
    else if( command < joint_urdf_[i]->limits->lower ) // below lower limit
    {
      command = joint_urdf_[i]->limits->lower;
 //     ROS_WARN_STREAM("reached lower limit: " << joint_[i].getName());
    }
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(custom_controller::ActuatorPositionController1, controller_interface::ControllerBase)
