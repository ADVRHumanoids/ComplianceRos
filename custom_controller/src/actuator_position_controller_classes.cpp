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

#include "custom_controller/actuator_position_controller_classes.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <math.h>

namespace custom_controller
{

ActuatorPositionControllerClasses::ActuatorPositionControllerClasses()
    : loop_count_(0)
{
}

ActuatorPositionControllerClasses::~ActuatorPositionControllerClasses()
{
  sub_command_.shutdown();
}

bool ActuatorPositionControllerClasses::init(
    hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n)
{
  //  code to publish the ROS_DEBUG informations to a console -- uncomment for
  //  debuging
  //  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //  ros::console::levels::Debug) ) {
  //	  ros::console::notifyLoggerLevelsChanged();}

  _getDofs(n);
  _initCommand();

  _loadedDofs(n);

  if (!_initControllers(robot, n))
    return false;
  if (!_initActuators(n))
    return false;

  _initDebugPrint(n);

  // check if controller had defined the update frequency
  if (!n.getParam("controller_update", _update_rate))
    _update_rate = 1;

  // Start realtime state publisher
  controller_debug_publisher_.reset(new realtime_tools::RealtimePublisher<
      custom_messages::GroupControllerMsg>(n, "debug", 1));
  controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, "state",
                                                                     1));
  // Start command subscriber
  sub_command_ = n.subscribe<custom_messages::CustomCmnd>(
      "command", 1, &ActuatorPositionControllerClasses::setCommandCB, this);

  update_gains_ = n.advertiseService(
      "update_gains", &ActuatorPositionControllerClasses::updateGains, this);
  update_print_ = n.advertiseService(
      "update_print", &ActuatorPositionControllerClasses::updatePrint, this);
  get_joint_names_ = n.advertiseService(
      "get_joint_names", &ActuatorPositionControllerClasses::getJointNames,
      this);
  set_ff_torque_ = n.advertiseService(
      "set_ff_torque", &ActuatorPositionControllerClasses::setFeedForwardTorque,
      this);

  return true;
}

bool ActuatorPositionControllerClasses::updateGains(
    custom_services::updateGains::Request& req,
    custom_services::updateGains::Response& res)
{
  std::stringstream msg;

  if (req.nr < 0 || req.nr > dof - 1)
  {
    res.success = false;
    msg << "Received number: " << req.nr
        << " is not defined within the controller. Please send a number "
           "bettween 0 and " << dof - 1 << "." << std::endl
        << "To see loaded joints call 'get_joint_names' service.";
    res.message = msg.str();
    return true;
  }

  res.success = true;

  if (req.a_p >= 0)
    _controllers.at(req.nr).a_Kp = req.a_p;
  else
  {
    msg << "Given actuator proportional gain is negative. Please give a "
           "positive number." << std::endl
        << "Actuator proportional gain not changed." << std::endl;
    res.success = false;
  }

  if (req.a_d >= 0)
    _controllers.at(req.nr).a_Kd = req.a_d;
  else
  {
    msg << "Given actuator derivative gain is negative. Please give a "
           "positive number." << std::endl
        << "Actuator derivative gain not changed." << std::endl;
    res.success = false;
  }

  if (req.j_p >= 0)
    _controllers.at(req.nr).j_Kp = req.j_p;
  else
  {
    msg << "Given joint proportional gain is negative. Please give a "
           "positive number." << std::endl
        << "Joint proportional gain not changed." << std::endl;
    res.success = false;
  }

  if (req.j_d >= 0)
    _controllers.at(req.nr).j_Kd = req.j_d;
  else
  {
    msg << "Given joint derivative gain is negative. Please give a positive "
           "number." << std::endl
        << "Joint derivative gain not changed." << std::endl;
    res.success = false;
  }
  if (req.i >= 0)
    _controllers.at(req.nr).Ki = req.i;
  else
  {
    msg << "Given integral gain is negative. Please give a positive number."
        << std::endl
        << "Integral gain not changed." << std::endl;
    res.success = false;
  }

  msg << "Changed controller gains for" << std::endl
      << "joint number: " << req.nr << std::endl
      << "joint name: " << _links.at(req.nr).name << "." << std::endl
      << "Current gains are: " << std::endl
      << "a_Kp: " << _controllers.at(req.nr).a_Kp << std::endl
      << "a_Kd: " << _controllers.at(req.nr).a_Kd << std::endl
      << "j_Kp: " << _controllers.at(req.nr).j_Kp << std::endl
      << "j_Kd: " << _controllers.at(req.nr).j_Kd << std::endl
      << "Ki: " << _controllers.at(req.nr).Ki << std::endl;
  res.message = msg.str();
  return true;
}

bool ActuatorPositionControllerClasses::updatePrint(
    custom_services::updatePrint::Request& req,
    custom_services::updatePrint::Response& res)
{
  if (req.command == "add")
  {
    res.message = "Added following joints of controller publisher\n";
    res.success = addPrint(req.nr, res.message);
  }
  else if (req.command == "remove")
  {
    res.success = removePrint(req.nr, res.message);

    res.message += printingList().str();
    res.success = true;
  }
  else if (req.command == "check")
  {

    res.message += "\n List of joints in controller publisher\n";
    res.message += printingList().str();

    for (int i = 0; i < _selector.size(); i++)
    {
      if (_controllers.at(i).print)
        res.ids.push_back(_selector.at(i));
    }

    res.success = true;
  }
  else
  {
    res.message = "unknown command value, send 'add' to publish joint states, "
                  " 'remove' to stop publishing joint data, and 'check' to "
                  "examine current state";
    res.success = false;
  }
}

std::stringstream ActuatorPositionControllerClasses::printingList()
{
  std::stringstream msg;

  msg << "\n List of joints in controller publisher\n";
  int k = 0;
  for (int i = 0; i < _controllers.size(); i++)
  {
    if (!_controllers.at(i).print)
      continue;

    msg << "printing number: " << k << "\t joint number: " << i
        << "\t joint name: " << _links.at(i).name << "." << std::endl;
    k++;
  }

  return msg;
}

bool ActuatorPositionControllerClasses::getJointNames(
    custom_services::getJointNames::Request& req,
    custom_services::getJointNames::Response& res)
{

  res.success = true;
  std::stringstream msg;
  msg << "joint number: \t joint name" << std::endl;

  for (int j = 0; j < dof; j++)
  {
    msg << _selector[j] << "\t" << _links.at(j).name << std::endl;
    res.joints.push_back(_links.at(j).name);
    res.ids.push_back(_selector[j]);
  }

  res.message = msg.str();

  return true;
}

// Set the actuator position command with a velocity command as well
void ActuatorPositionControllerClasses::setCommand(
    std::vector<double> pos_command, std::vector<double> vel_command,
    std::vector<double> eff_command, std::vector<double> gain1_command,
    std::vector<double> gain2_command)
{
  command_struct_.position_.clear();
  command_struct_.position_ = pos_command;

  command_struct_.velocity_.clear();
  command_struct_.velocity_ = vel_command;

  command_struct_.effort_.clear();
  command_struct_.effort_ = eff_command;

  command_struct_.onlineGain1_.clear();
  command_struct_.onlineGain1_ = gain1_command;

  command_struct_.onlineGain2_.clear();
  command_struct_.onlineGain2_ = gain2_command;

  command_.writeFromNonRT(command_struct_);
}

void ActuatorPositionControllerClasses::starting(const ros::Time& time)
{
  for (int i = 0; i < dof; i++)
  {
    _actuators.at(i).position = _links.at(i).interface.getPosition();
    _actuators.at(i).velocity = _links.at(i).interface.getVelocity();
    //      ROS_INFO_STREAM("joint" << i << " '" << joint_[i].getName() << "'
    //  / intialized at position " << joint_[i].getPosition() << " with " <<
    //  / joint_[i].getVelocity() <<" velocity" );
  }
  command_.initRT(command_struct_);
  loop_count_ = 0;
}

void ActuatorPositionControllerClasses::update(const ros::Time& time,
                                               const ros::Duration& period)
{

  command_struct_ = *(command_.readFromRT());
  //  double commanded_effort[dof];
  //  double error;
  double period_sec = period.toSec();
  bool update = !(loop_count_ % _update_rate);
  for (int i = 0; i < dof; i++)
  {

    // Get reference for this dof
    int real_id = _selector.at(i);

    double command_position = command_struct_.position_[real_id];
    // double command_velocity = command_struct_.velocity_[real_id];
    double command_effort = command_struct_.effort_[real_id];
    // double command_gain1 = command_struct_.onlineGain1_[real_id];
    // double command_gain2 = command_struct_.onlineGain2_[real_id];

    //    ///////////////////////////////////////////////////

    // Set ff torque to zero if disabled
    if (!ff_torque_enabled)
      command_effort = 0;

    // update_loop
    if (update)
    {
      _actuators.at(i).motor_torque = _controllers.at(i).update(
          command_position, command_effort, _actuators.at(i).position,
          _actuators.at(i).velocity);
    }

    _actuators.at(i).enforceTorqueLimit();

    _links.at(i).computeEffort(_actuators.at(i));

    _actuators.at(i).update(period_sec, _links.at(i).effort);

    /*
    // pd
        if (loop_count_ % _update_rate)
        {
          _links.at(i).effort = _controllers.at(i).update(
              command_position, command_effort,
    _links.at(i).interface.getPosition(),
              _links.at(i).interface.getVelocity());
        }
    */
  }

  for (int i = 0; i < dof; i++)
    _links.at(i).send();

  if (update)
  {
    _publishState(time);
    _publishDebug(time);
  }

  loop_count_++;
}

void ActuatorPositionControllerClasses::_publishDebug(const ros::Time& time)
{

  if (!controller_debug_publisher_)
    return;
  if (!controller_debug_publisher_->trylock())
    return;

  custom_messages::GroupControllerMsg pub_array;
  custom_messages::ControllerMsg pub;
  command_struct_ = *(command_.readFromRT());

  for (int i = 0; i < dof; i++)
  {
    if (!_controllers.at(i).print)
      continue;

    pub.link_side_position = _links.at(i).interface.getPosition();
    pub.link_side_velocity = _links.at(i).interface.getVelocity();
    pub.link_side_effort = _links.at(i).effort;

    pub.motor_side_position = _actuators.at(i).position;
    pub.motor_side_velocity = _actuators.at(i).velocity;
    pub.motor_side_effort = _actuators.at(i).motor_torque;

    pub.desired_position = command_struct_.position_[_selector.at(i)];
    pub_array.controller.push_back(pub);
  }

  controller_debug_publisher_->msg_.header.stamp = time;
  controller_debug_publisher_->msg_.controller = pub_array.controller;
  controller_debug_publisher_->unlockAndPublish();
}

void ActuatorPositionControllerClasses::_publishState(const ros::Time& time)
{

  if (!controller_state_publisher_)
    return;
  if (!controller_state_publisher_->trylock())
    return;

  sensor_msgs::JointState pub;
  for (int i = 0; i < dof; i++)
  {
    pub.position.push_back(_actuators.at(i).position);
    pub.velocity.push_back(_actuators.at(i).velocity);
    pub.effort.push_back(_actuators.at(i).motor_torque);
  }

  controller_state_publisher_->msg_.header.stamp = time;
  controller_state_publisher_->msg_.position = pub.position;
  controller_state_publisher_->msg_.velocity = pub.velocity;
  controller_state_publisher_->msg_.effort = pub.effort;

  controller_state_publisher_->unlockAndPublish();

  return;
}

void ActuatorPositionControllerClasses::Link::send()
{

  interface.setCommand(effort);
}

void ActuatorPositionControllerClasses::Link::enforceTorqueLimit(double limit)
{
  effort = (fabs(effort) < limit) ? effort : effort / fabs(effort) * limit;
}

void ActuatorPositionControllerClasses::Link::computeEffort(Actuator& actuator)
{
  effort = actuator.Ks * (actuator.position - interface.getPosition()) +
           actuator.D * (actuator.velocity - interface.getVelocity());

  enforceTorqueLimit(actuator.torque_limit);
}

double ActuatorPositionControllerClasses::Controller::update(
    double desired_position, double feed_forward, double position,
    double velocity)
{
  enforceStateLimits(desired_position);

  double error = desired_position - position;

  integrator_error += error;

  return feed_forward + Ki * integrator_error + a_Kp * error - a_Kd * velocity;
}

void ActuatorPositionControllerClasses::Actuator::enforceTorqueLimit()
{
  motor_torque = (fabs(motor_torque) < torque_limit)
                     ? motor_torque
                     : motor_torque / fabs(motor_torque) * torque_limit;
}

void ActuatorPositionControllerClasses::setCommandCB(
    const custom_messages::CustomCmnd msg)
{
  setCommand(msg.position, msg.velocity, msg.effort, msg.onlineGain1,
             msg.onlineGain2);
}

void ActuatorPositionControllerClasses::Controller::enforceStateLimits(
    double& command)
{
  if (type == JOINT_TYPE::CONTINUOUS) return;
  // Check that the desired positions are within the feasible limits
  if (command > upper_limit) // above upper limnit
  {
    command = upper_limit;
    //     ROS_WARN_STREAM("reached upper limit: " << joint_[ii].getName());
  }
  else if (command < lower_limit) // below lower limit
  {
    command = lower_limit;
    //     ROS_WARN_STREAM("reached lower limit: " << joint_[ii].getName());
  }
}

bool ActuatorPositionControllerClasses::addPrint(int data,
                                                 std::string& response)
{
  std::stringstream str;

  if (data < 0 || data > dof - 1)
  {
    str << "Received number: " << data
        << " is not defined within the controller. Please send a number "
           "bettween 0 and " << dof - 1 << ".\n"
        << "To see loaded joints call 'get_joint_names' service." << std::endl;
    response += str.str();
    return false;
  }

  if (_controllers.at(data).print)
  {
    str << "Requested joint nr " << data
        << " is already printed by the controller" << std::endl;
    response += str.str();
    return false;
  }

  _controllers.at(data).print = true;
  str << "joint " << data << ", joint_name: " << _links.at(data).name
      << std::endl;

  response += str.str();
  return true;
}

bool ActuatorPositionControllerClasses::removePrint(int data,
                                                    std::string& response)
{
  std::stringstream str;

  if (data < 0 || data > dof - 1)
  {
    str << "Received number: " << data
        << " is not defined within the controller. Please send a number "
           "bettween 0 and " << dof - 1 << ".\n"
        << "To see loaded joints call 'get_joint_names' service." << std::endl;
    response += str.str();
    return false;
  }
  if (!_controllers.at(data).print)
  {
    str << "Requested joint " << data
        << " has not been printed by the controller" << std::endl;
    response += str.str();
    return false;
  }

  _controllers.at(data).print = false;

  response += str.str();
  return true;
}

bool ActuatorPositionControllerClasses::setFeedForwardTorque(
    std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{

  std::string enabled = "enabled", disabled = "disabled";
  ff_torque_enabled = req.data;
  res.success = true;
  res.message = "Feedforward torque term was successfully " +
                (ff_torque_enabled ? enabled : disabled) + "!";
}

void ActuatorPositionControllerClasses::Actuator::update(
    double period_sec, double commanded_effort)
{
  acceleration = (motor_torque - phi * velocity - commanded_effort) / B;
  position +=
      0.5 * acceleration * period_sec * period_sec + velocity * period_sec;
  velocity += acceleration * period_sec;
}

void ActuatorPositionControllerClasses::_getDofs(ros::NodeHandle& n)
{

  if (!n.getParam("dof", dof))
  {
    ROS_ERROR_STREAM("Couldn't find number of joints defined for this "
                     "controller. Please define parameter"
                     << n.getNamespace().c_str() << "/dof.");
  }

  _selector.resize(dof);
  std::iota(_selector.begin(), _selector.end(), 0);

  _reversed_selector.resize(dof, -1);
}

bool ActuatorPositionControllerClasses::_loadedDofs(ros::NodeHandle& n)
{

  XmlRpc::XmlRpcValue omit_list;
  if (!n.getParam("omit", omit_list))
    return true;

  if (omit_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Wrong data type for parameter 'omit_list'.");
    return false;
  }

  std::stringstream str;
  str << "Ommits following joints " << n.getNamespace().c_str() << std::endl;

  for (int i = 0; i < omit_list.size(); i++)
  {

    if (omit_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
      continue;

    int omit = omit_list[i];

    --omit;

    auto exists = std::find_if(_selector.begin(), _selector.end(), [omit](int i)
                               {
                                 return i == omit;
                               });
    if (exists == _selector.end())
      continue;

    _selector.erase(exists);
    str << "joint" << omit + 1 << std::endl;
  }

  dof = _selector.size();

  // init reversed selector
  for (int i = 0; i < _selector.size(); i++)
    _reversed_selector.at(_selector[i]) = i;

  ROS_INFO_STREAM(str.str());
}

bool ActuatorPositionControllerClasses::_initActuators(ros::NodeHandle& n)
{

  std::string robot_namespace = n.getNamespace();
  robot_namespace.erase(robot_namespace.rfind("/") + 1);

  XmlRpc::XmlRpcValue actuator;
  double Ks, D, B, phi, nu, torque_limit;
  int select;
  for (int i = 0; i < _selector.size(); i++)
  {
    select = _selector[i];
    std::string name = _links.at(i).name;
    if (!n.getParam(robot_namespace + "actuators/" + name, actuator))
    {
      ROS_ERROR("Couldn't find actuators data for joint %s", name.c_str());
      return false;
    }

    if (!_loadDoubleParameter(Ks, "Ks", actuator, select + 1))
      return false;
    if (!_loadDoubleParameter(D, "D", actuator, select + 1))
      return false;
    if (!_loadDoubleParameter(phi, "phim", actuator, select + 1))
      return false;
    if (!_loadDoubleParameter(torque_limit, "Torque_limit", actuator,
                              select + 1))
      return false;

    nu = (actuator.hasMember("nu")) ? (int)actuator["nu"] : 1;

    if (actuator.hasMember("Bm"))
    {
      if (!_loadDoubleParameter(B, "Bm", actuator, select + 1))
        return false;
      B = B * nu * nu;
    }
    else if (actuator.hasMember("B"))
    {
      if (!_loadDoubleParameter(B, "B", actuator, select + 1))
        return false;
    }
    else
    {
      ROS_ERROR_STREAM("Motor inertia for " << name.c_str() << " undefined.");
      return false;
    }

    _actuators.push_back(Actuator(Ks, D, B, phi * nu * nu, torque_limit));
  }

  return true;
}

bool ActuatorPositionControllerClasses::_initControllers(
    hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n)
{

  XmlRpc::XmlRpcValue joint;
  double a_Kp, a_Kd, j_Kp, j_Kd, Ki;

  for (auto& i : _selector)
  {
    if (!n.getParam("joint" + std::to_string(i + 1), joint))
    {
      ROS_ERROR_STREAM("Couldn't find a description of a joint" << i + 1);
      return false;
    }

    if (!_loadDoubleParameter(a_Kp, "a_Kp", joint, i + 1))
      return false;
    if (!_loadDoubleParameter(a_Kd, "a_Kd", joint, i + 1))
      return false;
    if (!_loadDoubleParameter(j_Kp, "j_Kp", joint, i + 1))
      return false;
    if (!_loadDoubleParameter(j_Kd, "j_Kd", joint, i + 1))
      return false;
    if (!_loadDoubleParameter(Ki, "Ki", joint, i + 1))
      return false;

    _controllers.push_back(Controller(a_Kp, a_Kd, j_Kp, j_Kd, Ki));

    if (!joint.hasMember("name"))
    {
      ROS_ERROR_STREAM("Excpected parameter 'name' not defined in joint"
                       << i + 1);
      return false;
    }

    std::string name = joint["name"];
    _links.push_back(Link(name));
    _links.back().interface = robot->getHandle(name);
    // read joint limits from urdf
    urdf::Model urdf;

    if (!urdf.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    // Get URDF info about joint
    _controllers.back().lower_limit = urdf.getJoint(name)->limits->lower;

    _controllers.back().upper_limit = urdf.getJoint(name)->limits->upper;
    _controllers.back().type = static_cast<JOINT_TYPE>(urdf.getJoint(name)->type);
  }

  return true;
}

bool ActuatorPositionControllerClasses::_loadDoubleParameter(
    double& value, std::string param, XmlRpc::XmlRpcValue xml, int joint_number)
{

  if (!xml.hasMember(param))
  {
    ROS_ERROR_STREAM("Excpected parameter '"
                     << param << "' not defined in joint" << joint_number);
    return false;
  }
  // if type is int cast data to double

  if (xml[param].getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    int int_value = xml[param];
    value = int_value;
  }
  // if parameter is in double for assign data to controller
  else if (xml[param].getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    value = xml[param];
  }
  // if nonumeric parameter raise error
  else
  {
    ROS_ERROR_STREAM("Wrong parameter type: '"
                     << param << "' - excpected Integer or Double for joint"
                     << joint_number);
    return false;
  }
  // mark the actuator propotional gain has been assigned
  return true;
}

void ActuatorPositionControllerClasses::_initCommand()
{
  command_struct_.position_.resize(dof, 0);
  command_struct_.velocity_.resize(dof, 0);
  command_struct_.effort_.resize(dof, 0);
  command_struct_.onlineGain1_.resize(dof, 0);
  command_struct_.onlineGain2_.resize(dof, 0);
}

void ActuatorPositionControllerClasses::_initDebugPrint(ros::NodeHandle& n)
{
  // Initalize debug printing
  XmlRpc::XmlRpcValue print_list;
  if (!n.getParam("print_data", print_list))
    return;

  if (print_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN_STREAM("Received wrong data type for a debuging printing list");
    return;
  }

  std::stringstream str;
  str << "Print data for following joints of controller "
      << n.getNamespace().c_str() << std::endl;
  std::string message = str.str();
  ROS_INFO_STREAM(str.str());
  for (int j = 0; j < print_list.size(); j++)
  {

    if (print_list[j].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_WARN_STREAM("Wrong data type for parmaeter on a print list: " << j);
      continue;
    }

    int data = print_list[j];
    XmlRpc::XmlRpcValue joint;
    if (!n.getParam("joint" + std::to_string(data), joint))
    {
      ROS_WARN_STREAM("Couldn't retrivy data for joint "
                      << data << " from parameter server");
      continue;
    }

    if (!joint.hasMember("name"))
    {
      ROS_WARN_STREAM("Couldn't retrivy name of a joint "
                      << data << " from parameter server");
      continue;
    }

    std::string name = joint["name"];

    auto number = std::find_if(_links.begin(), _links.end(), [name](Link& link)
                               {
                                 return link.name == name;
                               });

    int nr = std::distance(_links.begin(), number);

    addPrint(nr, message);
  }

  ROS_INFO_STREAM(message);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(custom_controller::ActuatorPositionControllerClasses,
                       controller_interface::ControllerBase)
