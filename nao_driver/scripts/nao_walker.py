#!/usr/bin/env python

#
# ROS node to control Nao's walking engine (omniwalk and footsteps)
# This code is currently compatible to NaoQI version 1.6 or newer (latest
# tested: 1.12)
#
# Copyright 2009-2011 Armin Hornung & Stefan Osswald, University of Freiburg
# http://www.ros.org/wiki/nao
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

import roslib
roslib.load_manifest('nao_driver')
import rospy

from nao_driver import *

import math
from math import fabs

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import std_msgs.msg

from std_srvs.srv import Empty, EmptyResponse
from nao_msgs.srv import CmdPoseService, CmdVelService, CmdPoseServiceResponse, CmdVelServiceResponse
from humanoid_nav_msgs.msg import StepTarget
from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceResponse

import nao_msgs.srv

from start_walk_pose import startWalkPose

class NaoWalker(NaoNode):
    def __init__(self):
        NaoNode.__init__(self)

        # ROS initialization:
        rospy.init_node('nao_walker')

        self.connectNaoQi()

        # head scan is not active
        self.scan_active = False
        # walking pattern params:
        self.stepFrequency = rospy.get_param('~step_frequency', 0.5)

        self.useStartWalkPose = rospy.get_param('~use_walk_pose', False)
        self.needsStartWalkPose = True

        # other params
        self.maxHeadSpeed = rospy.get_param('~max_head_speed', 0.2)
        # initial stiffness (defaults to 0 so it doesn't strain the robot when no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        self.for_iros2011 = rospy.get_param('~for_iros2011', False)
        rospy.loginfo("for_iros2011 = %d" % self.for_iros2011)
        if self.for_iros2011:
            self.doubleScan = rospy.get_param('~double_scan', False)
            self.doubleScanFrom = rospy.get_param('~double_scan_from', 0.0)
            self.doubleScanTo = rospy.get_param('~double_scan_to', -30.0)
            self.scanInfoPub = rospy.Publisher("scan_info", Bool, latch=True)
            scanInfo = Bool(False)
            self.scanInfoPub.publish(scanInfo)


        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)

        try:
            enableFootContactProtection = rospy.get_param('~enable_foot_contact_protection')
            self.motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", enableFootContactProtection]])
            if enableFootContactProtection:
                rospy.loginfo("Enabled foot contact protection")
            else:
                rospy.loginfo("Disabled foot contact protection")
        except KeyError:
            # do not change foot contact protection
            pass

        # last: ROS subscriptions (after all vars are initialized)
        rospy.Subscriber("cmd_vel", Twist, self.handleCmdVel, queue_size=1)
        rospy.Subscriber("cmd_pose", Pose2D, self.handleTargetPose, queue_size=1)
        rospy.Subscriber("cmd_step", StepTarget, self.handleStep, queue_size=50)
        rospy.Subscriber("speech", String, self.handleSpeech)
        rospy.Subscriber("start_scan", std_msgs.msg.Empty, self.handleStartScan, queue_size=1)

        # ROS services (blocking functions)
        self.cmdPoseSrv = rospy.Service("cmd_pose_srv", CmdPoseService, self.handleTargetPoseService)
        self.cmdVelSrv = rospy.Service("cmd_vel_srv", CmdVelService, self.handleCmdVelService)
        self.stepToSrv = rospy.Service("cmd_step_srv", StepTargetService, self.handleStepSrv)
        self.headScanSrv = rospy.Service("head_scan_srv", Empty, self.handleHeadScanSrv)
        self.stopWalkSrv = rospy.Service("stop_walk_srv", Empty, self.handleStopWalkSrv)
        self.needsStartWalkPoseSrv = rospy.Service("needs_start_walk_pose_srv", Empty, self.handleNeedsStartWalkPoseSrv)

        self.say("Walker online")

        rospy.loginfo("nao_walker initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.getProxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

        self.ttsProxy = self.getProxy("ALTextToSpeech", warn=False)
        if self.ttsProxy is None:
            rospy.logwarn("No Proxy to TTS available, disabling speech output.")


    def stopWalk(self):
        """ Stops the current walking bahavior and blocks until the clearing is complete. """
        try:
            self.motionProxy.setWalkTargetVelocity(0.0, 0.0, 0.0, self.stepFrequency)
            self.motionProxy.waitUntilWalkIsFinished()


        except RuntimeError,e:
            print "An error has been caught"
            print e
            return False

        return True


    def handleSpeech(self,data):
        self.say(data.data)

    def say(self, text):
        if (not self.ttsProxy is None):
            try:
                # self.ttsProxy.say() sometimes causes deadlocks
                # see http://users.aldebaran-robotics.com/index.php?option=com_kunena&Itemid=14&func=view&catid=68&id=3857&limit=6&limitstart=6#6251
                print("say %s ..." % str(text))
                ttsid = self.ttsProxy.post.say(str(text))
                #self.ttsProxy.wait(ttsid, 0)
                print("...done")
            except RuntimeError,e:
                rospy.logerr("Exception caught:\n%s", e)

    def handleCmdVel(self, data):
        rospy.logdebug("Walk cmd_vel: %f %f %f, frequency %f", data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
        if data.linear.x != 0 or data.linear.y != 0 or data.angular.z != 0:
            self.gotoStartWalkPose()
        try:
            eps = 1e-3 # maybe 0,0,0 is a special command in motionProxy...
            if abs(data.linear.x)<eps and abs(data.linear.y)<eps and abs(data.angular.z)<eps:
                self.motionProxy.setWalkTargetVelocity(0,0,0,0.5)
            else:
                self.motionProxy.setWalkTargetVelocity(data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
        except RuntimeError,e:
            # We have to assume there's no NaoQI running anymore => exit!
            rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
            rospy.signal_shutdown("No NaoQI available anymore")



    def handleCmdVelService(self, req):
        self.handleCmdVel(req.twist)
        return CmdVelServiceResponse()

    def handleTargetPose(self, data, post=True):
        """handles cmd_pose requests, walks to (x,y,theta) in robot coordinate system"""
        if self.scan_active:
            return False

        rospy.logdebug("Walk target_pose: %f %f %f", data.x,
                data.y, data.theta)

        self.gotoStartWalkPose()

        try:
            if post:
                self.motionProxy.post.walkTo(data.x, data.y, data.theta )
            else:
                self.motionProxy.walkTo(data.x, data.y, data.theta )
            return True
        except RuntimeError,e:
            rospy.logerr("Exception caught in handleTargetPose:\n%s", e)
            return False


    def handleStep(self, data):
        rospy.logdebug("Step leg: %d; target: %f %f %f", data.leg, data.pose.x,
                data.pose.y, data.pose.theta)
        try:
            if data.leg == StepTarget.right:
                leg = "RLeg"
            elif data.leg == StepTarget.left:
                leg = "LLeg"
            else:
                rospy.logerr("Received a wrong leg constant: %d, ignoring step command", data.leg)
                return
            self.motionProxy.stepTo(leg, data.pose.x, data.pose.y, data.pose.theta)
            return True
        except RuntimeError, e:
            rospy.logerr("Exception caught in handleStep:\n%s", e)
            return False

    def handleStepSrv(self, req):
        if self.handleStep(req.step):
            return StepTargetServiceResponse()
        else:
            return None

    def handleTargetPoseService(self, req):
        """ do NOT use post"""
        if self.handleTargetPose(req.pose, False):
            return CmdPoseServiceResponse()
        else:
            return None

    def handleHeadScanSrv(self, req):
        if self.perform_laser_scan():
            return EmptyResponse()
        else:
            return None

    def handleStartScan(self, msg):
        self.perform_laser_scan()

    def handleStopWalkSrv(self, req):
        if self.stopWalk():
            return EmptyResponse()
        else:
            return None

    def gotoStartWalkPose(self):
        if self.useStartWalkPose and self.needsStartWalkPose:
            startWalkPose(self.motionProxy)
            self.needsStartWalkPose = False

    def handleNeedsStartWalkPoseSrv(self, data):
        self.needsStartWalkPose = True
        return EmptyResponse()


    def perform_laser_scan(self):
        if self.scan_active:
            print "ignoring button press because I'm already scanning"
            return False
        self.scan_active = True
        self.stopWalk()
        if self.for_iros2011:
            from iros2011_scan import head_scan
            head_scan(self.motionProxy,self.ttsProxy, 28.0, -36.0, 1.0, 10.0, True)
            if self.doubleScan:
                head_scan(self.motionProxy, self.ttsProxy, self.doubleScanFrom, self.doubleScanTo, 1.0, 10.0, False)
                self.motionProxy.angleInterpolation('HeadPitch', 0.0, 2.5, True)
        else:
            # todo: remove sleeps
            #from headscan import head_scan
            from headscanTop import head_scan
            if (not self.ttsProxy is None):
                self.say("Starting scan")
            else:
                rospy.loginfo("Starting scan")
            # max angle, min angle (pitch), scan freq, resolution, use squat
            rospy.sleep(2)
            #head_scan(self.motionProxy,26.0,-5.0,10.0,0.5,True)
            head_scan(self.motionProxy,28.0, -35.0, 1.0, 10.0, True)
            if (not self.ttsProxy is None):
                self.say("Scan complete")
            else:
                rospy.loginfo("Scan complete")
        self.scan_active = False
        self.needsStartWalkPose = True
        return True


if __name__ == '__main__':
    walker = NaoWalker()
    rospy.loginfo("nao_walker running...")
    rospy.spin()
    rospy.loginfo("nao_walker stopping...")
    walker.stopWalk()

    rospy.loginfo("nao_walker stopped.")
    exit(0)
