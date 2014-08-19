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

import logging

from rosgraph_msgs.msg import Log
import rospy

from nao_driver.nao_driver_naoqi import NaoNode

import qi

# level 0 is actually 'silent' in NAOqi and 5 is 'verbose' but we adapt to the ROS ones
LEVELS = [Log.DEBUG, Log.FATAL, Log.ERROR, Log.WARN, Log.INFO, Log.DEBUG, Log.DEBUG]
try:
    ROSOUT_PUB = rospy.topics.Publisher('/rosout', Log, latch=True, queue_size=100)
except:
    # Groovy compatible code
    ROSOUT_PUB = rospy.topics.Publisher('/rosout', Log, latch=True)

def onMessageCallback(msg):
    """
    Given a NAOqi dict message, publish it in /rosout
    :param msg: dict corresponding to a NAOqi log message
    """
    file, function, line = msg['source'].split(':')
    if line:
        line = int(line)
    # adapt the category to what a topic is (kindof).
    l = Log(level=LEVELS[msg['level']], name=str(msg['category']), msg=msg['message'], file=file, line=line, function=function)
    l.header.stamp = rospy.Time(msg['timestamp']['tv_sec'], msg['timestamp']['tv_usec'])
    ROSOUT_PUB.publish(l)

class NaoLogger(NaoNode):
    #This should be treated as a constant
    NODE_NAME = 'nao_logger'

    def __init__( self ):
        #Initialization
        NaoNode.__init__( self, self.NODE_NAME )

        from distutils.version import LooseVersion
        if self.get_version() < LooseVersion('2.0.0'):
            rospy.loginfo('The NAOqi version is inferior to 2.0, hence no log bridge possible')
            exit(0)

        rospy.init_node( self.NODE_NAME )

        # the log manager is only avaiable through a session (NAOqi 2.0)
        self.session = qi.Session()
        self.session.connect("tcp://%s:%s" % (self.pip, self.pport))
        self.logManager = self.session.service("LogManager")

        self.listener = self.logManager.getListener()
        self.listener.onLogMessage.connect(onMessageCallback)
        rospy.loginfo('Logger initialized')

if __name__ == '__main__':
    try:
        nao_logger = NaoLogger()
    except RuntimeError as e:
        rospy.logerr('Something went wrong: %s' % str(e) )

    rospy.spin()
