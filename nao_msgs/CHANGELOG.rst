^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2014-10-27)
------------------
* Addition of an audio message
* get the accent right in Séverin's name
* Contributors: Vincent Rabaud, sambrose

0.4.0 (2014-09-18)
------------------

0.3.0 (2014-08-19)
------------------
* update maintainers
  Armin, thank you for all your work, in ROS, Octomap and NAO.
  Good luck out of the university !
* Merge branch 'pose_manager_posture_enhancement' of https://github.com/etsardou/nao_robot into etsardou-pose_manager_posture_enhancement
  Conflicts:
  nao_msgs/CMakeLists.txt
* Moved the predefined pose support from pose_manager to nao_controller. Action name changed as well
* SpeechWithFeedback.action refined. Erased unneccessary bool flags.
* Support for predefined postures via actionlib
* Added actionlib interface for speech with feedback
* Added support for changing speech recognition vocabulary via actionlib. Updates the dynamic reconfigure variable as well.
* Adding service for toggling the arm motion while walking
* Contributors: Armin Hornung, Manos Tsardoulias, Vincent Rabaud

0.2.3 (2014-03-27)
------------------

0.2.2 (2013-10-28)
------------------

0.2.1 (2013-10-25)
------------------

0.2.0 (2013-10-25)
------------------
* Catkinization
* Removed unused messages TorsoIMU & TorsoOdometry, replaced by standard ROS msgs

0.1.0 (2013-08-01)
------------------
* Added node to interface Nao speech recognition and tts
* Added module to control NAO's LEDs
* more refactoring of nao stack
