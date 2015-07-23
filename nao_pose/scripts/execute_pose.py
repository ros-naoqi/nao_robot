#!/usr/bin/env python

# A simple action client calling the pose_manager
#
# Author: Armin Hornung, University of Freiburg

import rospy
import sys
import actionlib

from naoqi_bridge_msgs.msg import BodyPoseAction, BodyPoseGoal

if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('\nUSAGE: %s pose_name\n\n' %  sys.argv[0])

    rospy.init_node('execute_pose')

    poseClient = actionlib.SimpleActionClient("body_pose", BodyPoseAction)
    if not poseClient.wait_for_server(rospy.Duration(3.0)):
        rospy.logfatal("Could not connect to required \"body_pose\" action server, is the pose_manager node running?");
        rospy.signal_shutdown();

    goal = BodyPoseGoal
    goal.pose_name = str(sys.argv[1])
    rospy.loginfo("Calling pose_manager for pose %s...", goal.pose_name)


    poseClient.send_goal_and_wait(goal, rospy.Duration(5.0))
    #TODO: check for errors
    exit(0)
