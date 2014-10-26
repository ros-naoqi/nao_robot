^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2014-10-27)
------------------
* Fix qi/qitype dependency inconsistency
* Export FindNAOqi.cmake so that other projects can use the same file
* get the accent right in Séverin's name
* Turn odometry tf broadcast on/off
* Update nao_sensors.py
* Contributors: Jamie Diprose, Vincent Rabaud, jdddog, sambrose

0.4.0 (2014-09-18)
------------------
* Add 'queue_size=10' to Publisher, for Indigo's compatability
* Contributors: hris2003

0.3.0 (2014-08-19)
------------------
* update maintainers
  Armin, thank you for all your work, in ROS, Octomap and NAO.
  Good luck out of the university !
* respect PEP008
* mention nao_sensors instead of nao_camera
* Merge pull request `#29 <https://github.com/ros-nao/nao_robot/issues/29>`_ from severin-lemaignan/remove_camera
  Remove the Python camera driver
* [python] better threading for NaoNode (with ROS shutdown handling)
  also do a few import cleanups
* modify nao_driver for naoqi2 compatibility
* use proper Python indentation for several files
* Add a Logger bridge for NAOqi 2.0
* add a way to get the NAOqi version
* Added missing dependency on diagnostics in nao_driver cmake lists.
* Removed hard-coded ports from nao_sensors_cpp and nao_speech.py
* Modified nao_tactile node to search for a free port.
* Fixing PR `#23 <https://github.com/ros-nao/nao_robot/issues/23>`_ for simulation usage
  Use class NaoNode as before for proxy creation
* Fixing PR `#24 <https://github.com/ros-nao/nao_robot/issues/24>`_ for NaoQI 1.12
  ALRobotPosture is optional, node no longer exits
* make the license explicit as mentioned by @ahornung
* move NaoJointsAnalyzer.cpp from nao_viz
* run nao_diagnostic_updater by default
* Removed the Python camera driver
  It has been moved to nao_camera since a few months
* Added erroneous input checks. Altered the action server name.
* Moved the predefined pose support from pose_manager to nao_controller. Action name changed as well
* SpeechWithFeedback.action refined. Erased unneccessary bool flags.
* Double inheritance from NaoNode and ALModule implemented
* Added actionlib interface for speech with feedback
* Added support for changing speech recognition vocabulary via actionlib. Updates the dynamic reconfigure variable as well.
* Extend sim detection in nao_diagnostic_updater.py
* Adding service for toggling the arm motion while walking
* Contributors: Armin Hornung, Jon Dybeck, Manos Tsardoulias, Séverin Lemaignan, Vincent Rabaud, jondy276, margueda

* The Python camera driver has moved to `nao_camera <https://github.com/ros-nao/nao_sensors>`_

0.2.3 (2014-03-27)
------------------
* Fix dependency of nao_driver on camera_info_manager_py
* Fix nao_controller for incomplete NAOs
* Get nao_diagnostic_updater.py running again

0.2.2 (2013-10-28)
------------------
* Replace accented character in package.xml files, seems to cause
  problems with bloom

0.2.1 (2013-10-25)
------------------

0.2.0 (2013-10-25)
------------------
* Adding force_python parameter to nao_driver.launch to switch
  between C++ and python nodes for nao_sensors (Issue `#11 <https://github.com/ros-nao/nao_robot/issues/11>`_)
  Parameter will be passed on from higher-level launch files (nao_bringup).
  Default node is C++, unless a simulation launch file is started.
* Removing unnecessary output="screen" from launch files
* Add more nodes to the launch file
  This is related to bug `#11 <https://github.com/ros-nao/nao_robot/issues/11>`_
* Update rate parameter of nao_sensors (py/C++) is now called ~sensor_rate,
  25Hz default rate. Changed to proper rospy.Rate in python version.
* Small cleanup of nao_sensors_cpp / py
* nao_driver.launch defaults to nao_sensors.py instead of cpp
* Check rosparams as well as cmdline for ip and port
* Nao speech should now work with simulated naoqi
* Install nao_driver's config directory, removed cfg dir
* nao_camera.py changes, uses camera_info_manager_py now
* Imported services pause|resume|offset odometry to nao_sensors
  This complete the merge of remap_odometry in nao_sensors
* nao_sensors now exports directly std odometry
* nao_sensors now publish std ROS IMU, complete IMU message by also exporting angular velocities
  and linear acceleration. Note that these values require naoqi >= 1.14
* Enabled compilation of nao_sensors_cpp (when NaoQI found)
* Removed speech topic from nao_walker
* nao_speech now uses dynamic reconfigure to update parameters.
* Cleaned up python library headers in nao_driver
* Catkinization of nao_driver
* Move nao_driver's node to 'nodes' directory (as per rep-008).
* deleted further obsolete pose scripts (replaced by pose_manager)
* Cleanup: removed scan scripts / callbacks from nao_driver.
  Should end up in a different package using a combination of
  pose_manager and nao_controller

0.1.0 (2013-08-01)
------------------
* nao_controller now uses a lock-free polling implementation when running actionlib tasks
* added foot gait config parameters to walker, plus service to change it at runtime
* Improvements to nao_controller:
  * Actions can now be preempted!
  * Fixed race conditions when preemption is requested before task starts
  * Set aborted and preempted terminal states if required
* Removed topic prefixes from nao_speech
* Improved nao_leds:
  * Removed topic prefixes
  * Range of std_msgs/ColorRGBA should be between [0, 1]
  * Clearer finishing conditions
* Improved nao_behaviors:
  * Removed race conditions between preemption and execution
  * Added debug information
  * Removed topic prefixes
* Improved nao_behaviors:
  * Removed race conditions between preemption and execution
  * Added debug information
  * Removed topic prefixes
* Added node to interface Nao speech recognition and tts
* Added module to control NAO's LEDs
* launch files reorganized
* Patch from Issue `#6 <https://github.com/ros-nao/nao_robot/issues/6>`_: added nao_behaviors node, service and actionlib interface to execute behaviors (thx to Miguel S.)
* added a camera node nao_camera.py
* Improved compatibility of nao_controller / pose_manager with both H25 and H21 Naos.
  New script execute_pose in nao_remote to test.
* added driver launchfile for NaoQI simulation
* Added nao_diagnostic_updater node for diagnostic messages
* nao_common and nao_robot now compatible to REP-120
* nao_driver for using naoqi 1.12
* new footstep control script for NaoQI 1.12
* nao_sensors update: added cpp implementation
* Cleanup of nao_walker: got rid of a few deprecated scripts and MotionCommandBtn
* rename of nao_ctrl => nao_driver
