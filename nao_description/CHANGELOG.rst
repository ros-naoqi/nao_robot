^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2014-10-27)
------------------
* added camera and sonar to naoGazebo.xacro
* get the accent right in Séverin's name
* Contributors: Vincent Rabaud, margueda

0.4.0 (2014-09-18)
------------------
* urdf generated files, launchfiles and config files
  generated files updated
  updated mesh file path
  add xacro macro for fingers in naohands
  update nao.urdf based on new macros for fingers
  export with meshes
  remove useless imu frame
* Contributors: margueda

0.3.0 (2014-08-19)
------------------
* update maintainers
  Armin, thank you for all your work, in ROS, Octomap and NAO.
  Good luck out of the university !
* Contributors: Armin Hornung, Vincent Rabaud

0.2.3 (2014-03-27)
------------------
* Fix `#17 <https://github.com/ros-nao/nao_robot/issues/17>`_: add dependency on sensor_msgs in nao_description

0.2.2 (2013-10-28)
------------------

0.2.1 (2013-10-25)
------------------
* [nao_description] Added missing include dirs in CMakeLists

0.2.0 (2013-10-25)
------------------
* Added base_footprint node to nao_description, publishes footprint according
  to REP-120 (based on previous node nao_remote/remap_odometry). Fixes Issue `#10 <https://github.com/ros-nao/nao_robot/issues/10>`_.
* Moved nao_description from nao_common to nao_robot

