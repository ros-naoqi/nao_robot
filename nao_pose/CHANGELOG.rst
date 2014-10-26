^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_pose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2014-10-27)
------------------
* get the accent right in Séverin's name
* Contributors: Vincent Rabaud

0.4.0 (2014-09-18)
------------------

0.3.0 (2014-08-19)
------------------
* Fix xapparser __init__.py encoding, UTF used in header
* Moved the predefined pose support from pose_manager to nao_controller. Action name changed as well
* Support for predefined postures via actionlib
* Contributors: Armin Hornung, Manos Tsardoulias

0.2.3 (2014-03-27)
------------------

0.2.2 (2013-10-28)
------------------
* Replace accented character in package.xml files, seems to cause
  problems with bloom

0.2.1 (2013-10-25)
------------------

0.2.0 (2013-10-25)
------------------
* Added support for Choregraphe's XAP posture library
  This allows a user to create Nao posture from Choregraphe, export them to XAP files
  and load them in the pose library for later invokation with body_pose action.
* Add a parser for Choregraphe XAP posture libraries
* Import pose_manager from nao_remote into nao_robot/nao_pose
