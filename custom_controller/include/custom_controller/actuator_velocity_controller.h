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

#ifndef EFFORT_CONTROLLERS__ACTUATOR_VELOCITY_CONTROLLER_H
#define EFFORT_CONTROLLERS__ACTUATOR_VELOCITY_CONTROLLER_H

/**
   @class custom_controller::ActuatorVelocityController
   @brief ActuatorVelocityController

   This class controls velocity

   @section ROS ROS interface

   @PARAMETERS
   @param type: custom_controller/ActuatorVelocityController4 #(required)
   @param dof: number of controlled joints #(required)
   @param print_data: #(optional)
             - first published joint
             - second published joint
             -
             - nth published joint
   @param joint{jointnumber} (ex. joint1, joint2): #(required)
             name: joint name in urdf #(required)
             a_Kp: controller actuator proportional gain #(ignored)
             a_Kd: controller actuator derivative gain #(required)
             j_Kp: controller joint proportional gain #(ignored)
             j_Kd: controller joint derivative gain #(ignored)
             Ki: controller integral gain #(ignored)
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
#include <numeric>
		


namespace custom_controller
{

enum class JOINT_TYPE {

        REVOULTE = urdf::Joint::REVOLUTE,
        CONTINUOUS = urdf::Joint::CONTINUOUS,
        PRISMATIC = urdf::Joint::PRISMATIC
};

class ActuatorVelocityController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface>
{
public:
  // number of joints in controller
  // strcture to keep the desired positions
  struct Commands
  {
    std::vector<double> position_;    // Last commanded position
    std::vector<double> velocity_;    // Last commanded velocity
    std::vector<double> effort_;      // Last commanded effort
    std::vector<double> onlineGain1_; // Last commanded online gain 1
    std::vector<double> onlineGain2_; // Last commanded online gain 2
  };

  ActuatorVelocityController();
  ~ActuatorVelocityController();

  bool init(hardware_interface::EffortJointInterface* robot,
            ros::NodeHandle& n);

  //  // function to handle information outside the real-time loop
  void setCommand(std::vector<double> pos_command,
                  std::vector<double> vel_command,
                  std::vector<double> eff_command,
                  std::vector<double> gain1_command,
                  std::vector<double> gain2_command);

  //  // function called when the controller starts
  void starting(const ros::Time& time);

  //  // controller update function, called at each time step
  void update(const ros::Time& time, const ros::Duration& period);

  //  // controller services callbacks
  bool updateGains(custom_services::updateGains::Request& req,
                   custom_services::updateGains::Response& res);
  bool updatePrint(custom_services::updatePrint::Request& req,
                   custom_services::updatePrint::Response& res);
  bool getJointNames(custom_services::getJointNames::Request& req,
                     custom_services::getJointNames::Response& res);
  bool setFeedForwardTorque(std_srvs::SetBool::Request& req,
                            std_srvs::SetBool::Response& res);

  realtime_tools::RealtimeBuffer<Commands> command_;

  Commands command_struct_; // pre-allocated memory that is re-used to set the
                            // realtime buffer

private:
  int dof = 0;

  //  // number of iterations
  int loop_count_;

  //  // flag which enables/disables feedforward torque term
  bool ff_torque_enabled = false;

  // declare publisher
  boost::scoped_ptr<realtime_tools::RealtimePublisher<
      custom_messages::GroupControllerMsg>> controller_debug_publisher_;

  boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>
      controller_state_publisher_;

  // declare subscriber
  ros::Subscriber sub_command_;

  //  // declare services
  ros::ServiceServer update_gains_;
  ros::ServiceServer update_print_;
  ros::ServiceServer get_joint_names_;
  ros::ServiceServer set_ff_torque_;

  // structure to keep data related with actuators
  class Actuator
  {
  public:
    Actuator(double Ks, double D, double B, double phi, double torque_limit)
        : position(0), velocity(0), acceleration(0), motor_torque(0), Ks(Ks),
          D(D), B(B), phi(phi), torque_limit(torque_limit)
    {
    }
    ~Actuator() {}
    double position;
    double velocity;
    double acceleration;
    double torque_limit;
    double motor_torque;
    double Ks;
    double D;
    double B;
    double phi;
    void update(double period_sec, double commanded_effort);
    void enforceTorqueLimit();
    void computeCommand();

  };

  //  // structure to keep controller data
  class Controller
  {
  public:
    Controller(double a_Kd)
        : a_Kd(a_Kd), print(false)
    {}
    ~Controller() {}

    double a_Kd;
    bool print;
    double lower_limit; // I could implement velocity limit here
    double upper_limit; // I could implement velocity limit here
    JOINT_TYPE type;

    void enforceStateLimits(double &command);
    double update(double desired_velocity, double velocity);

  };

  class Link{
  public:
    Link(std::string name): name(name){}
    ~Link(){}
    hardware_interface::JointHandle interface;
    std::string name;
    double effort = 0;
    void computeEffort(Actuator& actuator);
    void enforceTorqueLimit(double limit);
    void send();
  };

  std::vector<Link> _links;
  std::vector<Controller> _controllers;
  std::vector<Actuator> _actuators;

  std::vector<int> _selector;
  std::vector<int> _reversed_selector;

  //  // subscriber callback function
  void setCommandCB(const custom_messages::CustomCmnd msg);
  //  // service update_print internal functions
  bool addPrint(int data, std::string& response);
  bool removePrint(int data, std::string& response);
  std::stringstream printingList();

  static bool loadDoubleParameter(std::string param, double& storage,
                                  XmlRpc::XmlRpcValue xml);

  bool _loadDoubleParameter(double& value, std::string param,
                            XmlRpc::XmlRpcValue xml, int joint_number);

  void _getDofs(ros::NodeHandle& n);

  bool _loadedDofs(ros::NodeHandle& n);

  bool _initControllers(hardware_interface::EffortJointInterface* robot,
                        ros::NodeHandle& n);
  bool _initActuators(ros::NodeHandle& n);
  bool _initLinks(ros::NodeHandle& n);

  void _initCommand();

  void _initDebugPrint(ros::NodeHandle& n);

  int _update_rate;

  void _publishState(const ros::Time& time);
  void _publishDebug(const ros::Time& time);
};

} // namespace

#endif
