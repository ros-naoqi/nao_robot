ROS-Stack for the Nao humanoid robot
http://code.google.com/p/alufr-ros-pkg/

A detailed and up-to date documentation of all of this stack's nodes 
is available online at:
  
  http://www.ros.org/wiki/nao
  
#########################################


  

This ROS stack contains two nodes which enable you to teleoperate a Aldebaran
Nao robot with a joystick and obtain some basic odometry estimate of the torso.

"nao_driver" is the control node which needs to be copied on the Nao, "nao_remote"
controls the Nao remotely. To run nao_driver on the Nao, you first need the Nao
ROS driver from Brown University: http://code.google.com/p/brown-ros-pkg/



After building nao_driver, copy the complete package (with the newly created
messages) into the "ros" directory of Brown's driver (i.e. naoros/ros). Edit
nao_driver/scripts/naoros.env on the robot appropriately, especially
ROS_MASTER_URI. To start the control node, have a ROS master running and execute 
on the robot (in naoros/ros/nao_driver/scripts/):

source naoros.env
./moveNao.py

By changing the command line parameters --pip and --pport, you can also connect
remotely to an Aldebaran Nao Broker running on a different machine (e.g. in a
Webots simulator).



For teleoperation, first compile the node with "rosmake nao_remote" and run:

roslaunch nao_remote teleop_nao.launch

This starts up the "joy" and "teleop_nao_joy" nodes. With a properly configured
gamepad, you can now teleoperate Nao. The controls are:

Right analog stick: translational velocity

Left analog stick (left / right): rotational velocity

D-pad: move head

Button 0: Stand up ("Init pose")

Button 8: Toggle gampad control on / off

Button 9: Go to safe "crouching" position and remove joint stiffness

Odometry is sent out as a basic "torso_odometry" message, which the
remap_odometry node remaps into a ROS odometry message.
