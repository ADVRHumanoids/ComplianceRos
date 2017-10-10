centauro-control package README description

--------
DESCRIPTION: contains all files related to a control of the robot. Controllers launch files, config parameters, and nodes to set desired positions.
--------

file description
--------
FOLDER config
    FILE centauro_actuators.xacro - contains a macro to add an actuator to a joint
    --------
    FILE centauro_control_comp_oneTopic.yaml - controller config file for an actuator simulation. Actuators are controllerd by PD controllers. Within this file a specific controller, its' gains and actuators parameters are defined.
    --------
    FILE centauro_control_comp_oneTopic_pd.yaml - controller config file for controlling the simulation without modelling actuators. Joints are controllerd by PD controllers. Within this file a specific controller and its' gains are defined.
--------
FOLDER launch
    FILE centauro_control_oneTopic.launch - launch file to start controllers and actuator simulation for a gazebo simulation. It calls the centauro_control_comp_oneTopic.yaml.
    INFO: play after a cantauro_world.launch has been executed
    ROS-COMMAND: roslaunch centauro_control centauro_control_oneTopic.launch
    --------
    FILE centauro_control_oneTopic_pd - launch file to start controllers for a gazebo simulation without simulating actuators. It calls the centauro_control_comp_oneTopic_pd.yaml.
    INFO: play after a cantauro_world.launch has been executed
    ROS-COMMAND: roslaunch centauro_control centauro_control_oneTopic_pd.launch 
--------
FOLDER nodes
    FILE centauro_setActuatorsComp_oneTopic - python script to simultaneously send commands to all controllers for actuators simulations
    ROS-COMMAND: rosrun centauro_control centauro_setActuatorsComp_oneTopic
    --------
    FILE centauro_setActuatorsComp_oneTopic_pd - python script to simultaneously send commands to all controllers for simple PD controller. 
    ROS-COMMAND: rosrun centauro_control centauro_setActuatorsComp_oneTopic_pd
    ROS-COMMAND: rosrun centauro_control centauro_setActuators
