#!/usr/bin/env python

#
# ROS node to control Peppers's move base
# This code is currently compatible to NaoQI version 1.6 or newer (latest
# tested: 1.12)
#
# Copyright 2014, Kei Okada
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

class NaoWheel(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_wheel')

        self.connectNaoQi()

        # last: ROS subscriptions (after all vars are initialized)
        rospy.Subscriber("cmd_vel", Twist, self.handleCmdVel, queue_size=1)
        rospy.Subscriber("cmd_pose", Pose2D, self.handleTargetPose, queue_size=1)

        # Create ROS publisher for speech
        self.pub = rospy.Publisher("speech", String, latch = True)

        self.say("wheel online")

        rospy.loginfo("nao_wheel initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

    def say(self, text):
        self.pub.publish(text)

    def handleCmdVel(self, data):
        rospy.logdebug("Wheel cmd_vel: %f %f %f", data.linear.x, data.linear.y, data.angular.z)
        try:
            eps = 1e-3 # maybe 0,0,0 is a special command in motionProxy...
            if abs(data.linear.x)<eps and abs(data.linear.y)<eps and abs(data.angular.z)<eps:
                self.motionProxy.moveToward(0,0,0)
            else:
                data.linear.x = max(min(1, data.linear.x), -1)
                data.linear.y = max(min(1, data.linear.y), -1)
                data.angular.z = max(min(1, data.angular.z), -1)
                rospy.loginfo("move %f, %f, %f" % (data.linear.x, data.linear.y, data.angular.z))
                self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z)
        except RuntimeError,e:
            # We have to assume there's no NaoQI running anymore => exit!
            rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
            rospy.signal_shutdown("No NaoQI available anymore")

    def handleTargetPose(self, data, post=True):
        """handles cmd_pose requests, move to (x,y,theta) in robot coordinate system"""
        rospy.logdebug("Wheel target_pose: %f %f %f", data.x, data.y, data.theta)
        try:
            return self.motionProxy.moveTo(data.x, data.y, data.theta)
        except RuntimeError,e:
            rospy.logerr("Exception caught in handleTargetPose:\n%s", e)
            return False




if __name__ == '__main__':
    wheel = NaoWheel()
    rospy.loginfo("nao_wheel running...")
    rospy.spin()
    rospy.loginfo("nao_wheel stopping...")
    exit(0)
