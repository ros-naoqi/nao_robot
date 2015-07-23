#!/usr/bin/env python
#
# ROS node to provide a few joint angle trajectories as default poses
#
# Copyright 2011 Armin Hornung & Daniel Maier University of Freiburg
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

import rospy
import sys

import rospy.rostime
from rospy.rostime import Duration

import actionlib
from actionlib_msgs.msg import GoalStatus
import naoqi_bridge_msgs.msg
from naoqi_bridge_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction, BodyPoseAction, BodyPoseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import xapparser

class PoseManager():
    def __init__(self):
        # ROS initialization:
        rospy.init_node('pose_manager')

        self.poseLibrary = dict()
        self.readInPoses()
        self.poseServer = actionlib.SimpleActionServer("body_pose", BodyPoseAction,
                                                       execute_cb=self.executeBodyPose,
                                                       auto_start=False)
        self.trajectoryClient = actionlib.SimpleActionClient("joint_trajectory", JointTrajectoryAction)
        if self.trajectoryClient.wait_for_server(rospy.Duration(3.0)):
            try:
                rospy.wait_for_service("stop_walk_srv", timeout=2.0)
                self.stopWalkSrv = rospy.ServiceProxy("stop_walk_srv", Empty)
            except:
                rospy.logwarn("stop_walk_srv not available, pose_manager will not stop the walker before executing a trajectory. "
                          +"This is normal if there is no nao_walker running.")
            self.stopWalkSrv = None
            self.poseServer.start()

            rospy.loginfo("pose_manager running, offering poses: %s", self.poseLibrary.keys());

        else:
            rospy.logfatal("Could not connect to required \"joint_trajectory\" action server, is the nao_controller node running?");
            rospy.signal_shutdown("Required component missing");


    def parseXapPoses(self, xaplibrary):
        try:
            poses = xapparser.getpostures(xaplibrary)
        except RuntimeError as re:
            rospy.logwarn("Error while parsing the XAP file: %s" % str(re))
            return

        for name, pose in poses.items():

            trajectory = JointTrajectory()

            trajectory.joint_names = pose.keys()
            joint_values = pose.values()

            point = JointTrajectoryPoint()
            point.time_from_start = Duration(2.0) # hardcoded duration!
            point.positions = pose.values()
            trajectory.points = [point]

            self.poseLibrary[name] = trajectory


    def readInPoses(self):

        xaplibrary = rospy.get_param('~xap', None)
        if xaplibrary:
            self.parseXapPoses(xaplibrary)

        poses = rospy.get_param('~poses', None)
        if poses:
            for key,value in poses.iteritems():
                try:
                # TODO: handle multiple points in trajectory
                    trajectory = JointTrajectory()
                    trajectory.joint_names = value["joint_names"]
                    point = JointTrajectoryPoint()
                    point.time_from_start = Duration(value["time_from_start"])
                    point.positions = value["positions"]
                    trajectory.points = [point]
                    self.poseLibrary[key] = trajectory
                except:
                    rospy.logwarn("Could not parse pose \"%s\" from the param server:", key);
                    rospy.logwarn(sys.exc_info())

        # add a default crouching pose:
        if not "crouch" in self.poseLibrary:
            trajectory = JointTrajectory()
            trajectory.joint_names = ["Body"]
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(1.5)
            point.positions = [0.0,0.0,                     # head
                1.545, 0.33, -1.57, -0.486, 0.0, 0.0,        # left arm
                -0.3, 0.057, -0.744, 2.192, -1.122, -0.035,     # left leg
                -0.3, 0.057, -0.744, 2.192, -1.122, -0.035,    # right leg
                1.545, -0.33, 1.57, 0.486, 0.0, 0.0]        # right arm
            trajectory.points = [point]
            self.poseLibrary["crouch"] = trajectory;


        rospy.loginfo("Loaded %d poses: %s", len(self.poseLibrary), self.poseLibrary.keys())


    def executeBodyPose(self, goal):
        if not goal.pose_name in self.poseLibrary:
            rospy.loginfo("Pose %s not in library, reload library from parameters..." % goal.pose_name)
            self.readInPoses()

        if goal.pose_name in self.poseLibrary:
            rospy.loginfo("Executing body pose %s...", goal.pose_name);
            if not self.stopWalkSrv is None:
                self.stopWalkSrv()

            trajGoal = JointTrajectoryGoal()
            # time out one sec. after trajectory ended:
            trajGoal.trajectory = self.poseLibrary[goal.pose_name]
            timeout = trajGoal.trajectory.points[-1].time_from_start + Duration(1.0)
            trajGoal.trajectory.header.stamp = rospy.Time.now()
            # TODO: cancel goal after timeout is not working yet in nao_controller
            self.trajectoryClient.send_goal_and_wait(trajGoal, timeout)
            if self.trajectoryClient.get_state() != GoalStatus.SUCCEEDED:
                self.poseServer.set_aborted(text="JointTrajectory action did not succeed (timeout?)");

            self.poseServer.set_succeeded()
            rospy.loginfo("Done.");

        else:
            msg = "pose \""+goal.pose_name+"\" not in pose_manager's library";
            rospy.logwarn("%s", msg)
            self.poseServer.set_aborted(text=str(msg));



if __name__ == '__main__':

    manager = PoseManager()
    rospy.spin()
    exit(0)

