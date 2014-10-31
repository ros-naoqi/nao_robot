#!/usr/bin/env python
#
# ROS node to control Nao's footsteps (testing for NaoQI 1.12)
#
# Copyright 2012 Armin Hornung and Johannes Garimort, University of Freiburg
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

import rospy
import time

from nao_driver import NaoNode

import math
from math import fabs

import actionlib

from humanoid_nav_msgs.msg import *
from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceResponse
from humanoid_nav_msgs.srv import ClipFootstep, ClipFootstepResponse

from nao_driver.util import ( startWalkPose, clip_footstep_tuple )


LEG_LEFT = "LLeg"
LEG_RIGHT = "RLeg"
FLOAT_CMP_THR = 0.000001
STEP_TIME = 0.5


def float_equ(a, b):
    return abs(a - b) <= FLOAT_CMP_THR


# redefine StepTarget by inheritance
class StepTarget(StepTarget):
    def __init__(self, x=0.0, y=0.0, theta=0.0, leg=0.0):
        super(StepTarget, self).__init__()
        self.pose.x = round(x, 4)
        self.pose.y = round(y, 4)
        self.pose.theta = round(theta, 4)
        self.leg = leg

    def __eq__(self, a):
        return (float_equ(self.pose.x, a.pose.x) and
                float_equ(self.pose.y, a.pose.y) and
                float_equ(self.pose.theta, a.pose.theta) and
                self.leg == a.leg)

    def __ne__(self, a):
        return not (self == a)

    def __str__(self):
        return "(%f, %f, %f, %i)" % (self.pose.x, self.pose.y, self.pose.theta,
                                     self.leg)

    def __repr__(self):
        return self.__str__()


class NaoFootsteps(NaoNode):
    def __init__(self):
        NaoNode.__init__(self, 'nao_footsteps')

        self.connectNaoQi()

        # initial stiffness (defaults to 0 so it doesn't strain the robot when
        # no teleoperation is running)
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
        """(re-) connect to NaoQI"""
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)


    def stopWalk(self):
        """
        Stops the current walking bahavior and blocks until the clearing is
        complete.
        """
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
                rospy.logerr("Received a wrong leg constant: %d, ignoring step",
                             " command", data.leg)
                return

            footStep = [[data.pose.x, data.pose.y, data.pose.theta]]
            timeList = [STEP_TIME]
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
        def update_feedback(feedback, executed_footsteps):
            # check if an footstep has been performed
            if not len(executed_footsteps):
                return
            # use the last footstep in the list since this might be the new one
            # (NOTE: if one step is missed here, increase the feedback rate)
            leg, time, (x, y, theta) = executed_footsteps[-1]
            # check if footstep information is up-to-date
            if not float_equ(time, STEP_TIME):
                return
            leg = (StepTarget.right if leg == LEG_RIGHT else
                   StepTarget.left)
            step = StepTarget(x, y, theta, leg)
            # add the footstep only if it is a new one
            try:
                if feedback.executed_footsteps[-1] == step:
                    return
            except IndexError:
                pass
            feedback.executed_footsteps.append(step)

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
            steps.append([round(step.pose.x, 4),
                          round(step.pose.y, 4),
                          round(step.pose.theta, 4)])
            try:
                time_list.append(time_list[-1] + STEP_TIME)
            except IndexError:
                time_list.append(STEP_TIME)

        rospy.loginfo("Start executing footsteps %s",
                      [[x, y, theta, leg] for (x, y, theta), leg in
                       zip(steps, legs)])

        feedback = ExecFootstepsFeedback()
        result = ExecFootstepsResult()
        success = True
        self.motionProxy.setFootSteps(legs, steps, time_list, True)
        while self.motionProxy.walkIsActive():
            # handle preempt requests
            if self.actionServer.is_preempt_requested():
                self.motionProxy.stopWalk()
                self.actionServer.set_preempted()
                rospy.loginfo("Preempting footstep execution");
                success = False
                break

            # get execution information from the robot and update the feedback
            (_, executed_footsteps, _) = self.motionProxy.getFootSteps()
            update_feedback(feedback, executed_footsteps)
            self.actionServer.publish_feedback(feedback)

            rospy.Rate(goal.feedback_frequency).sleep()

        if success:
            result.executed_footsteps = feedback.executed_footsteps
            self.actionServer.set_succeeded(result)


if __name__ == '__main__':

    walker = NaoFootsteps()
    rospy.loginfo("nao_footsteps running...")
    rospy.spin()
    rospy.loginfo("nao_footsteps stopping...")
    walker.stopWalk()

    rospy.loginfo("nao_footsteps stopped.")
    exit(0)
