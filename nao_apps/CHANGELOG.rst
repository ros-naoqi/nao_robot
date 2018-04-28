^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_apps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.15 (2016-11-23)
-------------------
* add missing dependencies
* renamed tacticle to tactile
* [nao_apps] Fixing bug generated when renaming in naoqi_bridge_msgs TactileTouch to HeadTouch
* Fix hard-coded node name in dynamic_reconfigure.client, closes `#26 <https://github.com/ros-naoqi/nao_robot/issues/26>`_
* Contributors: Felip Marti, Mikael Arguedas, Stefan Osswald

0.5.14 (2016-01-23)
-------------------

0.5.13 (2016-01-16)
-------------------

0.5.12 (2016-01-01)
-------------------
* change from naoqi_driver.cfg to naoqi_driver_py.cfg
* Contributors: Kanae Kochigami

0.5.11 (2015-08-11)
-------------------

0.5.10 (2015-07-31)
-------------------

0.5.9 (2015-07-30)
------------------

0.5.8 (2015-07-30)
------------------
* transfer to naoqi_py
* use naoqi_pose instead of nao_pose
* Contributors: Karsten Knese, Kei Okada

0.5.7 (2015-03-27)
------------------
* properly install Python scripts
  This fixes `#19 <https://github.com/ros-naoqi/nao_robot/issues/19>`_
* no need for custom file
* enable goto initial pose to start walking
* Contributors: Karsten Knese, Vincent Rabaud

0.5.6 (2015-02-27)
------------------
* Cleanup and rename launch files
* Contributors: Karsten Knese

0.5.5 (2015-02-17)
------------------

0.5.4 (2015-02-17)
------------------

0.5.3 (2014-12-14)
------------------
* check naoqi version and exit if not supported
* add nodes/nao_alife.py launch/nao_alife.launch
* add .launch files
* Contributors: Kei Okada

0.5.2 (2014-12-04)
------------------
* remove trailing spaces
* add nao_speech.launch
* Contributors: Kanae Kochigami, Mikael ARGUEDAS

0.5.1 (2014-11-13)
------------------
* Merge pull request `#2 <https://github.com/ros-naoqi/nao_robot/issues/2>`_ from k-okada/encode_utf8
  vocabulary must encode to utf8
* bugfix: missing reconfigure in naoqi_driver
* vocabulary must encode to utf8
* bugfix: python imports
* Contributors: Karsten Knese, Kei Okada, Vincent Rabaud

0.5.0 (2014-11-06)
------------------
* cleanups
* transfer nao_robot apps
* merge from nao_driver into nao_apps
* Contributors: Karsten Knese, Vincent Rabaud
