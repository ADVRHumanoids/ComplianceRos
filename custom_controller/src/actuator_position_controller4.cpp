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

#include "custom_controller/actuator_position_controller4.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <math.h>

namespace custom_controller {

ActuatorPositionController4::ActuatorPositionController4()
  : loop_count_(0)
{}

ActuatorPositionController4::~ActuatorPositionController4()
{
  sub_command_.shutdown();
}

bool ActuatorPositionController4::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
//  code to publish the ROS_DEBUG informations to a console -- uncomment for debuging
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	  ros::console::notifyLoggerLevelsChanged();}
  
  // reserve memory for declared number of joints
	actuator_.position.reserve(dof);
	actuator_.velocity.reserve(dof);
	actuator_.acceleration.reserve(dof);
	actuator_.torque_limit.reserve(dof);
	actuator_.motor_torque.reserve(dof);
	actuator_.Ks.reserve(dof);
	actuator_.D.reserve(dof);
	actuator_.B.reserve(dof);
	actuator_.phi.reserve(dof);
	controller_.a_Kp.reserve(dof);
	controller_.a_Kd.reserve(dof);
	controller_.j_Kp.reserve(dof);
	controller_.j_Kd.reserve(dof);
	controller_.Ki.reserve(dof);
	controller_.integrator_error.reserve(dof);
	controller_.print_list.reserve(dof);
	controller_.print_data.reserve(dof);

	joint_.reserve(dof);
  command_struct_.position_.reserve(dof);
  for (int i=0; i<dof; i++) {
    command_struct_.position_.push_back(0);
    actuator_.position[i] = 0;
    actuator_.velocity[i] = 0;
    actuator_.acceleration[i] = 0;
    actuator_.motor_torque[i] = 0;
    controller_.integrator_error[i] = 0;}
// check if number of joints is defined on parameter server
  if (!n.getParam("dof", dof)) 
  {
    ROS_ERROR("Not number of joints given 'dof' (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  
  for (int i=0; i<dof; i++)
	controller_.joints.push_back(i);

  
  XmlRpc::XmlRpcValue omit_list;
  if (n.getParam("omit", omit_list)){
	if(omit_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
		std::stringstream str;
		std::string message;
		str << "Ommits following joints " <<n.getNamespace().c_str() << std::endl;
		message = str.str();
		ROS_INFO_STREAM(str.str());
		for (int j=0; j<omit_list.size(); j++){
           if (omit_list[j].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int omit = omit_list[j];  
				for (int check_omit=0; check_omit<controller_.joints.size(); check_omit++){
					if (controller_.joints[check_omit] == omit-1){
						controller_.joints.erase(controller_.joints.begin()+check_omit);
						str << "joint" << omit <<  std::endl;break;
					}
				}
			}
		}
		message = str.str();
		ROS_INFO_STREAM(message);
	}

  }



 
    dof = controller_.joints.size();       
// Load parameters for each joint
  for (int ii=0; ii<dof; ii++){

  int i = controller_.joints[ii]; 
  // inialiaze actuator data	  

  // falg to verify if all parameters has been defined
  bool verify[12] = {false, false, false, false, false, false, false, false, false, false, false, false};

  std::stringstream j;
  j << "joint" << i+1 ;

  // read joints data
  XmlRpc::XmlRpcValue joint_list;
  if (!n.getParam(j.str(), joint_list)) 
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  // check if joints are defined in a structure
  if(joint_list.getType() == XmlRpc::XmlRpcValue::TypeStruct){
	  double nu;
  	  // iterate through joint structure
	  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = joint_list.begin(); it != joint_list.end(); ++it)   {
		std::string value = (std::string)(it->first) ;
		
		// check if given parameter is an actuator propotional gain   
		if( value == "a_Kp"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int a_Kp = joint_list[it->first];
		        controller_.a_Kp[i] = a_Kp;
		        }
			// if parameter is in double for assign data to controller
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			    controller_.a_Kp[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'a_Kp' - excpected Integer or Double");
				return false;
			}
			// mark the actuator propotional gain has been assigned
			verify[0] = true;	
			} 
		// check if given parameter is an actuator derivative gain 
		else if( value == "a_Kd"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int a_Kd = joint_list[it->first];
		        controller_.a_Kd[i] = a_Kd;
		        }
			// if parameter is in double for assign data to controller
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			    controller_.a_Kd[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'a_Kd' - excpected Integer or Double");
				return false;
				}
			// mark the actuator derivative gain has been assigned
			verify[1] = true;	
		}   
		// check if given parameter is a joint propotional gain   
		else if( value == "j_Kp"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int j_Kp = joint_list[it->first];
		        controller_.j_Kp[i] = j_Kp;
		        }
			// if parameter is in double for assign data to controller
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			    controller_.j_Kp[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'j_Kp' - excpected Integer or Double");
				return false;
			}
			// mark the joint propotional gain has been assigned
			verify[2] = true;	
			} 
		// check if given parameter is a joint derivative gain   
		else if( value == "j_Kd"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int j_Kd = joint_list[it->first];
		        controller_.j_Kd[i] = j_Kd;
		        }
			// if parameter is in double for assign data to controller
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			    controller_.j_Kd[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'j_Kd' - excpected Integer or Double");
				return false;
				}
			// mark the joint derivative gain has been assigned
			verify[3] = true;	
		}
		// check if given parameter is an intergal gain     
		else if( value == "Ki"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int Ki = joint_list[it->first];
		        controller_.Ki[i] = Ki;
		        }
			// if parameter is in double for assign data to controller
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
				controller_.Ki[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'Ki' - excpected Integer or Double");
				return false;
				}
			// mark the joint intergal has been assigned
			verify[4] = true;	
		}
		// check if given parameter is an actuator spring stiffness  		
		else if( value == "Ks"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int Ks = joint_list[it->first];
		        actuator_.Ks[i] = Ks;
		        }
			// if parameter is in double for assign data to actuator
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
				actuator_.Ks[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'Ks' - excpected Integer or Double");
				return false;
				}
			// mark the actuator spring stiffness has been assigned
			verify[5] = true;	
		}
		// check if given parameter is an actuator spring dumping  		
		else if( value == "D"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int D = joint_list[it->first];
		        actuator_.D[i] = D;
		        }
			// if parameter is in double for assign data to actuator
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			     actuator_.D[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'D' - excpected Integer or Double");
				return false;
				}
			// mark the actuator spring dumping has been assigned
			verify[6] = true;	
		}
		// check if given parameter is an actuator inertia 		
		else if( value == "Bm"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int Bm = joint_list[it->first];
		        actuator_.B[i] = Bm;
		        }
			// if parameter is in double for assign data to actuator
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
				actuator_.B[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'Bm' - excpected Integer or Double");
				return false;
				}
			// mark the actuator inertia has been assigned
			verify[7] = true;	
		}
		// check if given parameter is an actuator damping 		
		else if( value == "phim"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int phim = joint_list[it->first];
		        actuator_.phi[i] = phim;
		        }
			// if parameter is in double for assign data to actuator
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
				actuator_.phi[i] = joint_list[it->first];
			// if nonumeric parameter raise error
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'phim' - excpected Integer or Double");
				return false;
				}
			// mark the actuator damping has been assigned
			verify[8] = true;	
		}
		// check if given parameter is a gear ratio		
		else if( value == "nu"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int i_nu = joint_list[it->first];
		        nu = i_nu;
		        }
			// if parameter is in double for assign data to actuator
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			    nu = joint_list[it->first];
			// if nonumeric parameter raise error		
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'nu' - excpected Integer or Double");
				return false;
				}
			// mark the gear ratio has been assigned
			verify[9] = true;	
		}
		// check if given parameter is a motor torque limit		
			else if( value == "Torque_limit"){
			// if type is int cast data to double 
			if(joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int torque_limit = joint_list[it->first];
		        actuator_.torque_limit[i] = torque_limit;
		        }
			// if parameter is in double for assign data to actuator
		    	else if (joint_list[it->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
			    actuator_.torque_limit[i] = joint_list[it->first];
			// if nonumeric parameter raise error		
			else{
				ROS_ERROR_STREAM("Wrong parameter type: 'Torque_limit' - excpected Integer or Double");
				return false;
				}
			// mark the motor torque limit has been assigned
			verify[10] = true;	
		}
		// check if given parameter is a joint name		
			else if ( value == "name"){
				// if type is string read joint data from urdf file 	
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
										
					if (!joint_urdf_[ii]){
						ROS_ERROR_STREAM("Could not find joint in urdf" << joint_name << " " << i << " " << ii);
						return false;
					}
					
		        }
		        else{
					ROS_ERROR_STREAM("Wrong parameter type: 'Name' - excpected String"); 
					return false;
				}
			// mark the joint name has been assigned  
		        verify[11] = true;	
		        }
			// if different parameters raise a warning
			else ROS_WARN_STREAM("unknow parameter: '" << value << "' in definition of " <<  j.str());

  }
    // assign actuator output data
    actuator_.B[i] *= nu*nu;
    actuator_.phi[i] *= nu*nu;
    
    // check if all necessary parameters has been assigned if yes continue initalization, if no raise an error
    if(!verify[0]) {ROS_ERROR_STREAM("Excpected parameter 'a_Kd' not defined in " << j.str()); return false;}
    if(!verify[1]) {ROS_ERROR_STREAM("Excpected parameter 'a_Kp' not defined in " << j.str()); return false;}
    if(!verify[2]) {ROS_ERROR_STREAM("Excpected parameter 'j_Kd' not defined in " << j.str()); return false;}
    if(!verify[3]) {ROS_ERROR_STREAM("Excpected parameter 'j_Kp' not defined in " << j.str()); return false;}
 	if(!verify[4]) {ROS_ERROR_STREAM("Excpected parameter 'Ki' not defined in " << j.str()); return false;}   
    if(!verify[5]) {ROS_ERROR_STREAM("Excpected parameter 'Ks' not defined in " << j.str()); return false;}
    if(!verify[6]) {ROS_ERROR_STREAM("Excpected parameter 'D' not defined in " << j.str()); return false;}
    if(!verify[7]) {ROS_ERROR_STREAM("Excpected parameter 'Bm' not defined in " << j.str()); return false;}
    if(!verify[8]) {ROS_ERROR_STREAM("Excpected parameter 'phim' not defined in " << j.str()); return false;}
    if(!verify[9]) {ROS_ERROR_STREAM("Excpected parameter 'nu' not defined in " << j.str()); return false;}
    if(!verify[10]) {ROS_ERROR_STREAM("Excpected parameter 'Torque_limit' not defined in " << j.str()); return false;}
    if(!verify[11]) {ROS_ERROR_STREAM("Excpected parameter 'name' not defined in " << j.str()); return false;}    

  }
  else
    ROS_ERROR_STREAM("Wrong .yaml structure");
  }
/*
    ROS_DEBUG_STREAM("Ks1:" << actuator_.Ks[0] << " Ks2:" << actuator_.Ks[1]);
    ROS_DEBUG_STREAM("B1:" << actuator_.B[0] << "B2:" << actuator_.B[1]);
    ROS_DEBUG_STREAM("a_Kp1:" << controller_.a_Kp[0] << " a_Kp2:" << controller_.a_Kp[1]);
    ROS_DEBUG_STREAM("D1:" << actuator_.D[0] << "B2:" << actuator_.D[1]);
    ROS_DEBUG_STREAM("a_Kd1:" << controller_.a_Kd[0] << " a_Kd2:" << controller_.a_Kd[1]);
    ROS_DEBUG_STREAM("phi1:" << actuator_.phi[0] << "phi2:" << actuator_.phi[1]);
    ROS_DEBUG_STREAM("Ki1:" << controller_.Ki[0] << " Ki2:" << controller_.Ki[1]);
    ROS_DEBUG_STREAM("Torque_limit1:" << actuator_.torque_limit[0] << "Torque_limit2:" << actuator_.torque_limit[1]);
    ROS_DEBUG_STREAM("j_Kp1:" << controller_.a_Kp[0] << " j_Kp2:" << controller_.j_Kp[1]);
    ROS_DEBUG_STREAM("j_Kd1:" << controller_.a_Kd[0] << " j_Kd2:" << controller_.j_Kd[1]);
*/

  // Check0 which joint data to print
  XmlRpc::XmlRpcValue print_list;
  if (!n.getParam("print_data", print_list)) 
    ROS_WARN("None printing joint specify in: %s", n.getNamespace().c_str());
 
  if(print_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
	if (print_list.size() == 0)
	   ROS_WARN_STREAM("None printing joint specify in: " << n.getNamespace().c_str() << std::endl);
	else{
		std::stringstream str;
		std::string message;
		str << "Print data for following joints of controller " <<n.getNamespace().c_str() << std::endl;
		message = str.str();
		ROS_INFO_STREAM(str.str());
		for (int j=0; j<print_list.size(); j++){
			if (print_list[j].getType() == XmlRpc::XmlRpcValue::TypeInt){
				int data = print_list[j];
				addPrint(data, &message);
				ROS_INFO_STREAM(message);
				}
			else
				ROS_WARN_STREAM("Received wrong print data" << print_list[j]);
		}

		}
	}
  // check if controller_defined
  if (!n.getParam("controller_update", controller_.update)) 
  {
    controller_.update = 1;
  }
	
   // Start realtime state publisher
  controller_debug_publisher_.reset(
    new realtime_tools::RealtimePublisher<custom_messages::GroupControllerMsg>(n, "debug", 1));
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, "state", 1));
  // Start command subscriber
  sub_command_ = n.subscribe<sensor_msgs::JointState >("command", 1, &ActuatorPositionController4::setCommandCB, this);

  update_gains_ = n.advertiseService("update_gains", &ActuatorPositionController4::updateGains, this);
  update_print_ = n.advertiseService("update_print", &ActuatorPositionController4::updatePrint, this);
  get_joint_names_ = n.advertiseService("get_joint_names", &ActuatorPositionController4::getJointNames, this);
  
  return true;
}

bool ActuatorPositionController4::updateGains(custom_services::updateGains::Request &req, custom_services::updateGains::Response &res){
	
	if (req.nr > -1 & req.nr < dof){
		std::stringstream msg;
		res.success = true;
		if (req.a_p >= 0)
			controller_.a_Kp[req.nr] = req.a_p;
		else{
			msg << "Given actuator proportional gain is negative. Please give a positive number." << std::endl << "Actuator proportional gain not changed." << std::endl;
			res.success = false;
			}
		
		if (req.a_d >= 0)
			controller_.a_Kd[req.nr] = req.a_d;
		else{
			msg << "Given actuator derivative gain is negative. Please give a positive number." << std::endl << "Actuator derivative gain not changed." << std::endl;
			res.success = false;
			}
		if (req.j_p >= 0)
			controller_.j_Kp[req.nr] = req.j_p;
		else{
			msg << "Given joint proportional gain is negative. Please give a positive number." << std::endl << "Joint proportional gain not changed." << std::endl;
			res.success = false;
			}
		if (req.j_d >= 0)
			controller_.j_Kd[req.nr] = req.j_d;
		else{
			msg << "Given joint derivative gain is negative. Please give a positive number." << std::endl << "Joint derivative gain not changed." << std::endl;
			res.success = false;
			}		
		if (req.i >= 0)
			controller_.Ki[req.nr] = req.i;
		else{
			msg << "Given integral gain is negative. Please give a positive number." << std::endl << "Integral gain not changed." << std::endl;
			res.success = false;
			}
		
		
		msg << "Changed controller gains for" << std::endl << "joint number: " << req.nr << std:: endl << "joint name: " << joint_[req.nr].getName() << "." << std::endl << "Current gains are: " << std::endl << "a_Kp: " << controller_.a_Kp[req.nr] << std::endl << "a_Kd: " << controller_.a_Kd[req.nr] << std::endl << "j_Kp: " << controller_.j_Kp[req.nr] << std::endl << "j_Kd: " << controller_.j_Kd[req.nr]<< "Ki: " << controller_.Ki[req.nr] << std::endl;
		res.message = msg.str();	
		return true;}
	else{
		res.success = false;
		std::stringstream msg;
		msg  << "Received number: " <<  req.nr << " is not defined within the controller. Please send a number bettween 0 and " << dof-1 << "." << std::endl << "To see loaded joints call 'get_joint_names' service.";
		res.message = msg.str();		
		return true;}
	}
	
bool ActuatorPositionController4::updatePrint(custom_services::updatePrint::Request &req, custom_services::updatePrint::Response &res){
	if (req.command == "add") {
		res.message = "Added following joints of controller publisher\n";
		res.success = addPrint(req.nr, &res.message);
		}
	else if (req.command == "remove"){
		res.success = removePrint(req.nr, &res.message);
		std::stringstream msg;
		res.message += "\n List of joints in controller publisher\n";
		for (int i = 0; i < controller_.print_list.size(); i++){
			msg << "printing number: " << i << "\t joint number: " << controller_.print_list[i] << "\t joint name: " << joint_[controller_.print_data[i]].getName() << "." << std::endl;
		}
		res.message += msg.str();
		res.success = true;
		}
	else if (req.command == "check"){
		 std::stringstream msg;
		 res.message += "\n List of joints in controller publisher\n";
		 for (int i = 0; i < controller_.print_list.size(); i++){
			msg << "printing number: " << i << "\t joint number: " << controller_.print_list[i] << "\t joint name: " << joint_[controller_.print_data[i]].getName() << "." << std::endl;
		}
		res.message += msg.str();
		res.success = true;
		}
	else {res.message = "unknown command value, send 'add' to publish joint states and 'remove' to stop publishing joint data";
		res.success = false;}

}
	
bool ActuatorPositionController4::getJointNames(custom_services::getJointNames::Request &req, custom_services::getJointNames::Response &res){
	
	res.success = true;
	std::stringstream msg;
	msg << "joint number: \t joint name" << std::endl; 

	for (int j=0; j<dof; j++){
		msg << j <<  "\t" << joint_[j].getName() << std::endl; 
	}
	
	res.message = msg.str();		

	return true;
	}

// Set the actuator position command with a velocity command as well
void ActuatorPositionController4::setCommand(std::vector<double> pos_command)
{
  command_struct_.position_.clear();
  command_struct_.position_ = pos_command;
  
  command_.writeFromNonRT(command_struct_);
}

void ActuatorPositionController4::starting(const ros::Time& time)
{
  for (int ii=0; ii<dof; ii++){
	int i = controller_.joints[ii]; 
    actuator_.position[i] = joint_[ii].getPosition();
    actuator_.velocity[i] = joint_[ii].getVelocity();
//    ROS_INFO_STREAM("joint" << i << " '" << joint_[i].getName() << "' intialized at position " << joint_[i].getPosition() << " with " << joint_[i].getVelocity() <<" velocity" );
  }
  command_.initRT(command_struct_);
  loop_count_ = 0;
}

void ActuatorPositionController4::update(const ros::Time& time, const ros::Duration& period)
{

  command_struct_ = *(command_.readFromRT());	
  double commanded_effort[dof];
  double error;
  double period_sec = period.toSec();

  for (int ii=0; ii<dof; ii++){
	int i = controller_.joints[ii]; 
  //commanded_effort[i] = 0;

  double command_position = command_struct_.position_[i];

/////////////////////////////////////////////////
/*
    enforceJointLimits(command_position, i);
    error =  command_position - joint_[i].getPosition();
    controller_.integrator_error[i] += error;

    commanded_effort[i] = controller_.Ki[i]*controller_.integrator_error[i] + controller_.a_Kp[i]*error - controller_.a_Kd[i]*joint_[i].getVelocity();
    commanded_effort[i] = (fabs(commanded_effort[i]) < actuator_.torque_limit[i]) ? commanded_effort[i] : commanded_effort[i]/fabs(commanded_effort[i])*actuator_.torque_limit[i];
   actuator_.position[i] = joint_[i].getPosition();
   actuator_.velocity[i] = joint_[i].getVelocity();

if(i == controller_.print_data && controller_debug_publisher_ && controller_debug_publisher_->trylock()){
      controller_debug_publisher_->msg_.header.stamp =  time;
      controller_debug_publisher_->msg_.set_point = actuator_.velocity[i];
      controller_debug_publisher_->msg_.process_value =  actuator_.position[i];
      controller_debug_publisher_->msg_.process_value_dot =  joint_[i].getPosition();
      controller_debug_publisher_->msg_.error = motor_torque;
      controller_debug_publisher_->msg_.time_step = joint_[i].getVelocity();
      controller_debug_publisher_->msg_.command = commanded_effort[i];

      controller_debug_publisher_->unlockAndPublish();

}*/
    
    enforceJointLimits(command_position, ii);
    error =  command_position - actuator_.position[i];
	controller_.integrator_error[i] += error;

	actuator_.motor_torque[i] =  (loop_count_%controller_.update) ? actuator_.motor_torque[i] : (controller_.Ki[i]*controller_.integrator_error[i] + controller_.a_Kp[i]*error - controller_.a_Kd[i]*actuator_.velocity[i]);
    actuator_.motor_torque[i] = (fabs(actuator_.motor_torque[i]) < actuator_.torque_limit[i]) ? actuator_.motor_torque[i] : actuator_.motor_torque[i]/fabs(actuator_.motor_torque[i])*actuator_.torque_limit[i];

    commanded_effort[ii] = actuator_.Ks[i]*(actuator_.position[i]-joint_[ii].getPosition()) + actuator_.D[i]*(actuator_.velocity[i]-joint_[ii].getVelocity());
    commanded_effort[ii] = (fabs(commanded_effort[ii]) < actuator_.torque_limit[i]) ? commanded_effort[ii] : commanded_effort[ii]/fabs(commanded_effort[ii])*actuator_.torque_limit[i];

    actuator_.acceleration[i] = (actuator_.motor_torque[i]-actuator_.phi[i]*actuator_.velocity[i] - commanded_effort[ii])/actuator_.B[i];
    actuator_.position[i] += 0.5*actuator_.acceleration[i]*period_sec*period_sec + actuator_.velocity[i]*period_sec;
    actuator_.velocity[i] += actuator_.acceleration[i]*period_sec;
    }

	for (int ii=0; ii<dof; ii++)
		joint_[ii].setCommand(commanded_effort[ii]);
		
	if(!(loop_count_%controller_.update) && controller_debug_publisher_ && controller_debug_publisher_->trylock()){
		custom_messages::GroupControllerMsg pub_array;
		custom_messages::ControllerMsg pub;
	    command_struct_ = *(command_.readFromRT());	

		for (int ii = 0; ii < controller_.print_list.size(); ii++){
					
			int i = controller_.print_list[ii]; 
			int j = controller_.print_data[ii]; 
				
			pub.link_side_position = joint_[j].getPosition();
			pub.link_side_velocity = joint_[j].getVelocity();

			pub.motor_side_position = actuator_.position[controller_.joints[i]];
			pub.motor_side_velocity = actuator_.velocity[controller_.joints[i]];
			pub.link_side_effort = commanded_effort[j];
			pub.desired_position = command_struct_.position_[controller_.joints[i]];
			pub.motor_side_effort = actuator_.motor_torque[controller_.joints[i]];
			pub_array.controller.push_back(pub);}
			
		controller_debug_publisher_->msg_.header.stamp =  time;
		controller_debug_publisher_->msg_.controller = pub_array.controller;
		controller_debug_publisher_->unlockAndPublish();
	}
	
  if(!(loop_count_%controller_.update) && controller_state_publisher_ && controller_state_publisher_->trylock()){
	  sensor_msgs::JointState pub;
	  for (int ii = 0; ii < dof; ii++){
		   int i = controller_.joints[ii]; 
		pub.position.push_back(actuator_.position[i]);
		pub.velocity.push_back(actuator_.velocity[i]);
		pub.effort.push_back(actuator_.motor_torque[i]);
		}
      controller_state_publisher_->msg_.header.stamp =  time;
      controller_state_publisher_->msg_.position = pub.position;
      controller_state_publisher_->msg_.velocity = pub.velocity;
      controller_state_publisher_->msg_.effort = pub.effort;

      controller_state_publisher_->unlockAndPublish();
  }
	
  loop_count_++;
}

void ActuatorPositionController4::setCommandCB(const sensor_msgs::JointState  msg)
{
  setCommand(msg.position);
}

void ActuatorPositionController4::enforceJointLimits(double &command, int i)
{
  // Check that this joint has applicable limits
  if (joint_urdf_[i]->type == urdf::Joint::REVOLUTE)
  {
    if( command > joint_urdf_[i]->limits->upper ) // above upper limnit
    {
      command = joint_urdf_[i]->limits->upper;
 //     ROS_WARN_STREAM("reached upper limit: " << joint_[ii].getName());
    }
    else if( command < joint_urdf_[i]->limits->lower ) // below lower limit
    {
      command = joint_urdf_[i]->limits->lower;
 //     ROS_WARN_STREAM("reached lower limit: " << joint_[ii].getName());
    }
  }
}

bool ActuatorPositionController4::addPrint(int data, std::string* response){
	std::stringstream str;
	bool success = true;
	bool check = true;
	for (int check_print=0; check_print<controller_.print_list.size(); check_print++){
		if (controller_.print_list[check_print] == data){
			check = false;
			break;}
		}
	if(check){check = false;
		for (int check_omit=0; check_omit<controller_.joints.size(); check_omit++){
			if (controller_.joints[check_omit]==data){
				controller_.print_data.push_back(check_omit);
				controller_.print_list.push_back(data);
				str << "joint" << data << ", joint_name: " << joint_[check_omit].getName() << std::endl;
				check = true;
			break;
			}
		}
		if(!check) str << "joint" << data  << " is not defined within the controller." << std::endl;
				
	}
	else{success = false;
		str << "Received joint number: " <<  data << " is already printed by the controller"<< std::endl;}	
		
	*response += str.str();
	return success;
  }

bool ActuatorPositionController4::removePrint(int data, std::string* response){
	std::stringstream str;
	bool success = true;
	bool check = false;
		for (int check_print=0; check_print<controller_.print_list.size(); check_print++){
			if (controller_.print_list[check_print] == data){
				controller_.print_list.erase(controller_.print_list.begin()+check_print);
				controller_.print_data.erase(controller_.print_data.begin()+check_print);
				check = true;	
				str << "Succesfully removed joint number " << data << " from controller printer";				
				break;}
			}
		if(!check){success = false;
			str << "Received joint number: " <<  data << " is not printed by the controller"<< std::endl;}
  		*response += str.str();
	return success;
  }
} // namespace

PLUGINLIB_EXPORT_CLASS(custom_controller::ActuatorPositionController4, controller_interface::ControllerBase)
