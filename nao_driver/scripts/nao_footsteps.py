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

import actionlib

from humanoid_nav_msgs.msg import *
from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceResponse
from humanoid_nav_msgs.srv import ClipFootstep, ClipFootstepResponse

from start_walk_pose import startWalkPose
from nao_footstep_clipping import clip_footstep_tuple


LEG_LEFT = "LLeg"
LEG_RIGHT = "RLeg"
FLOAT_CMP_THR = 0.000001


def equal(a, b):
    return abs(a - b) <= FLOAT_CMP_THR


# redefine StepTarget by inheritance
class StepTarget(StepTarget):
    def __eq__(self, a):
        return (equal(self.pose.x, a.pose.x) and
                equal(self.pose.y, a.pose.y) and
                equal(self.pose.theta, a.pose.theta) and
                self.leg == a.leg)

    def __str__(self):
        return "(%f, %f, %f, %i)" % (self.pose.x, self.pose.y, self.pose.theta, self.leg)


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
        self.stepToSrv = rospy.Service("footstep_srv", StepTargetService,
                                       self.handleStepSrv)
        self.clipSrv = rospy.Service("clip_footstep_srv", ClipFootstep,
                                     self.handleClipSrv)

    	# Initialize action server
        self.actionServer = actionlib.SimpleActionServer(
            "footsteps_execution",
            ExecFootstepsAction,
            execute_cb=self.footstepsExecutionCallback,
            auto_start=False)
        self.actionServer.start()

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
                leg = [LEG_RIGHT]
            elif data.leg == StepTarget.left:
                leg = [LEG_LEFT]
            else:
                rospy.logerr("Received a wrong leg constant: %d, ignoring step command", data.leg)
                return

            footStep = [[data.pose.x, data.pose.y, data.pose.theta]]
            timeList = [0.5]
            self.motionProxy.setFootSteps(leg, footStep, timeList, False)
            time.sleep(0.1)
            print self.motionProxy.getFootSteps()
            self.motionProxy.waitUntilWalkIsFinished()

            return True
        except RuntimeError, e:
            rospy.logerr("Exception caught in handleStep:\n%s", e)
            return False

    def handleStepSrv(self, req):
        if self.handleStep(req.step):
            return StepTargetServiceResponse()
        else:
            return None

    def handleClipping(self, step):
        is_left_support = step.leg != StepTarget.left
        unclipped_step = (step.pose.x, step.pose.y, step.pose.theta)
        step.pose.x, step.pose.y, step.pose.theta = clip_footstep_tuple(
            unclipped_step, is_left_support)
        return step

    def handleClipSrv(self, req):
        resp = ClipFootstepResponse()
        resp.step = self.handleClipping(req.step)
        return resp

    def footstepsExecutionCallback(self, goal):
        def extend_footsteps_set(footstep_set, executed_footsteps):
            if not len(executed_footsteps):
                return
            leg, time, (x, y, theta) = executed_footsteps[-1]
            # check if footstep information is up-to-date
            if not equal(time, 0.5):
                return
            step = StepTarget()
            step.pose.x = round(x, 6)
            step.pose.y = round(y, 6)
            step.pose.theta = round(theta, 6)
            step.leg = (StepTarget.right if leg == LEG_RIGHT else
                        StepTarget.left)
            # if the step equals the last one that was added ignore it
            try:
                if footstep_set[-1] == step:
                    return
            except IndexError:
                pass
            footstep_set.append(step)

        legs = []
        steps = []
        time_list = []
        for step in goal.footsteps:
            if step.leg == StepTarget.right:
                legs.append(LEG_RIGHT)
            elif step.leg == StepTarget.left:
                legs.append(LEG_LEFT)
            else:
                rospy.logerr("Received a wrong leg constant: %d, ignoring step "
                             "command", step.leg)
                return
            steps.append([step.pose.x, step.pose.y, step.pose.theta])
            try:
                time_list.append(time_list[-1] + 0.5)
            except IndexError:
                time_list.append(0.5)

        steps = [[round(x, 4), round(y, 4), round(theta, 4)]
                 for x, y, theta in steps]
        rospy.loginfo("Start executing footsteps %s", steps)
        self.motionProxy.setFootSteps(legs, steps, time_list, True)

        feedback_rate = rospy.Rate(goal.feedback_rate)
        feedback = ExecFootstepsFeedback()
        result = ExecFootstepsResult()
        success = True
        footsteps_set = []
        while self.motionProxy.walkIsActive():
            if self.actionServer.is_preempt_requested():
                rospy.loginfo("Preempt footstep execution");
                self.motionProxy.stopWalk()
                self.actionServer.set_preempted()
                success = False
                break

            _ , executed_footsteps, _ = self.motionProxy.getFootSteps()
            extend_footsteps_set(footsteps_set, executed_footsteps)
            feedback.executed_footsteps = footsteps_set
            self.actionServer.publish_feedback(feedback)

            feedback_rate.sleep()

        if success:
            rospy.loginfo("Num. executed footsteps: %i", len(footsteps_set))
            result.executed_footsteps = footsteps_set
            self.actionServer.set_succeeded(result)


if __name__ == '__main__':

    walker = NaoFootsteps()
    rospy.loginfo("nao_footsteps running...")
    rospy.spin()
    rospy.loginfo("nao_footsteps stopping...")
    walker.stopWalk()

    rospy.loginfo("nao_footsteps stopped.")
    exit(0)
