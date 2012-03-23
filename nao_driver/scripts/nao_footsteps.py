#!/usr/bin/env python

# SVN $HeadURL$
# SVN $Id$


#
# ROS node to control Nao's footsteps (testing for NaoQI 1.12)
#
# Copyright 2012 Armin Hornung, University of Freiburg
# http://www.ros.org/wiki/nao_driver
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
import time

from nao_driver import *

import math
from math import fabs

from humanoid_nav_msgs.msg import StepTarget
from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceResponse

from start_walk_pose import startWalkPose

class NaoFootsteps(NaoNode):
    def __init__(self): 
        NaoNode.__init__(self)
    
        # ROS initialization:
        rospy.init_node('nao_footsteps')
        
        self.connectNaoQi()
    
        # initial stiffness (defaults to 0 so it doesn't strain the robot when no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)
    
        
        # last: ROS subscriptions (after all vars are initialized)
        rospy.Subscriber("footstep", StepTarget, self.handleStep, queue_size=50)
        
        # ROS services (blocking functions)
        self.stepToSrv = rospy.Service("footstep_srv", StepTargetService, self.handleStepSrv)
            
        rospy.loginfo("nao_footsteps initialized")
    
    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.getProxy("ALMotion")
        if self.motionProxy is None:
            exit(1)


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
        
    
    def handleStep(self, data):
        rospy.loginfo("Step leg: %d; target: %f %f %f", data.leg, data.pose.x,
                data.pose.y, data.pose.theta)
        try:
            if data.leg == StepTarget.right:
                leg = ["RLeg"]
            elif data.leg == StepTarget.left:
                leg = ["LLeg"]
            else:
                rospy.logerr("Received a wrong leg constant: %d, ignoring step command", data.leg)
                return
                
            footStep = [[data.pose.x, data.pose.y, data.pose.theta]]
            timeList = [2.5]
            self.motionProxy.setFootSteps(leg, footStep, timeList, False)
            time.sleep(0.1)
            print self.motionProxy.getFootSteps()
            #self.motionProxy.waitUntilWalkIsFinished()
            
            
            return True
        except RuntimeError, e:
            rospy.logerr("Exception caught in handleStep:\n%s", e)
            return False

    def handleStepSrv(self, req):
        if self.handleStep(req.step):
            return StepTargetServiceResponse()
        else:
            return None
    


if __name__ == '__main__':

    walker = NaoFootsteps()
    rospy.loginfo("nao_footsteps running...")
    rospy.spin()
    rospy.loginfo("nao_footsteps stopping...")
    walker.stopWalk()
    
    rospy.loginfo("nao_footsteps stopped.")
    exit(0)
