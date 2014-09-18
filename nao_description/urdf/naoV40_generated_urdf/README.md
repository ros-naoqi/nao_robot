Automatically generated urdf models
====================================

This folder contains xacro files automatically generated from aldebaran documentation. 
Because the generator is not complete yet, several files (hands and gazebo ones) have been made by hand based on the work of Konstantinos Chatzilygeroudis (https://github.com/costashatz/nao_dcm)

These files have been generated using : 
rosrun nao_description generate_urdf.py -i PATH/TO/THE/OFFICIAL/ALDEBARAN/URDF

You can generate the urdf file either by adding -x urdf or by running rosrun xacro xacro.py naorobot.xacro > nao.urdf

You can vizualise the robot using : 
    RVIZ : roslaunch nao_description display_generated.launch

TODO:
=====

generate automatically :
- naosensors.xacro (including every sensor not only sonars and camera)
- naohands.xacro (still missing the fingers)
- naoTransmission.xacro
- naoGazebo.xacro
All these file are currently edited by hand, the generator can generate part of them but none of them entirely 
