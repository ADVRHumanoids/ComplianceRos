custom_controller package README description

--------
DESCRIPTION: controller defined for a CENTAURO simulation based on the ros_controllers package, using ros_controll. Controllers from this package are general purpose and can be used with any gazebo and ros_control simulation.
	Currently two controllers are defined:
		a) simple PD effort based joint position controller working for multiple joints
		b) PD controller simulating Series Elastic Actuator, controller controlls a position of an actuator. Works for multiple joints.
	
--------

file description
--------
PLUGIN 
    TYPE custom_controller
    NAME ActuatorPositionController1
    INFO simple PD effort based joint position controller working for multiple joints
    SUBSCRIBES
	 description: receives desired position
	 topic: command
	 message: sensor_msgs/JointState
	 data:
		header
			secs
			nsecs
		name[] 		- omitted
		position[] 	- desired position
		velocity[] 	- omitted
		effort[]	- omitted
    PUBLISHES
    SERVICES	
	name: update_gains
	description: update controller gains
	request:
		p - proportional gain
		d - derivative gain
	response:
		success - service status
		message - detailed information
	--------
	name: get_joint_names
	description: gives information about controlled joints and its assigned numbers
	request:
		NONE
	response:
		success - service status
		message - detailed information
    PARAMETERS
	 controller_namespace:
	   type: custom_controller/ActuatorPositionController1 #(required)
	   dof: number of controlled joints #(required)
	   print_data: #(optional)
	     - first published joint
             - second published joint   
             - 
             - nth published joint   
	   joint{jointnumber} (ex. joint1, joint2): #(required)
	     name: joint name in urdf #(required)
	     Kp: controller proportional gain #(required)
	     Kd: controller derivative gain #(required)

--------
PLUGIN 
    TYPE custom_controller
    NAME ActuatorPositionController4
    INFO PD controller simulating Series Elastic Actuator, controller controlls a position of an actuator. Works for multiple
    SUBSCRIBES
	 description: receives desired position
	 topic: command
	 message: sensor_msgs/JointState
	 data:
		header
			secs
			nsecs
		name[] 		- omitted
		position[] 	- desired position
		velocity[] 	- omitted
		effort[]	- omitted
    PUBLISHES
	 description: publishes information about controller state
	 topic: state
	 message: custom_messages/GroupControllerMsg
	 data:
		header
			secs
			nsecs
		controller[]
			joint_position
			joint_velocity
			actuator_position
			actuator_velocity
			joint_effort
			commanded_effort
    SERVICES
	name: update_gains
	description: update controller gains
	request:
		a_p - actuator proportional gain
		a_d - actuator derivative gain
		j_p - joint proportional gain -- unused
		j_d - joint derivative gain -- unused
		i - integral gain -- unused
	response:
		success - service status
		message - detailed information
	--------
	name: update_print
	description: change logging properties
	request:
		nr - number of joint to update
		command - defined tha action
			add - starts printing information from joint number nr
			remove - stops printing information from joint number nr
			check - gives information about printing joints: joint number, joint name and assign printing number
	response:
		success - service status
		message - detailed information
	--------
	name: get_joint_names
	description: gives information about controlled joints and its assigned numbers
	request:
		NONE
	response:
		success - service status
		message - detailed information

    PARAMETERS
	 controller_namespace:
	   type: custom_controller/ActuatorPositionController4 #(required)
	   dof: number of controlled joints #(required)
	   print_data: #(optional)
	     - first published joint
             - second published joint   
             - 
             - nth published joint  
	   joint{jointnumber} (ex. joint1, joint2): #(required)
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

