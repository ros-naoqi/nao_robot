#!/usr/bin/env python

# Copyright (C) 2014 Aldebaran Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#import ROS dependencies
import rospy

#import NAO dependencies
from naoqi_sensors.ros_sonar import SonarSensor, SonarPublisher

if __name__ == '__main__':
    # create two sonars
    leftSonar = SonarSensor('Device/SubDeviceList/US/Left/Sensor/Value',     # AL memory key
                            'LSonar_frame',                                        # ROS frame id
                            '~/nao_robot/sonar/left')                              # ROS topic to publish

    rightSonar = SonarSensor('Device/SubDeviceList/US/Right/Sensor/Value',   # AL memory key
                             'RSonar_frame',                                       # ROS frame id
                             '~/nao_robot/sonar/right')                            # ROS topic to publish

    publisher = SonarPublisher( (leftSonar,rightSonar))                      # list of sonars
    publisher.start()

    rospy.spin()
    exit(0)
