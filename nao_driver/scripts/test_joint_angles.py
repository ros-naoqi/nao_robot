#!/usr/bin/env python

import roslib
#from rospy.exceptions import ROSException
roslib.load_manifest('nao_driver')
import rospy
import roslib.rostime
from roslib.rostime import Duration


import actionlib
import nao_msgs.msg
import trajectory_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
import std_srvs.srv

# go to crouching position
def joint_angle_client():
	#inhibitWalkSrv = rospy.ServiceProxy("inhibit_walk", std_srvs.srv.Empty)
	#uninhibitWalkSrv = rospy.ServiceProxy("uninhibit_walk", std_srvs.srv.Empty)
	
	client = actionlib.SimpleActionClient("joint_trajectory", nao_msgs.msg.JointTrajectoryAction)
	stiffness_client = actionlib.SimpleActionClient("joint_stiffness_trajectory", nao_msgs.msg.JointTrajectoryAction)
	angle_client = actionlib.SimpleActionClient("joint_angles_action", nao_msgs.msg.JointAnglesWithSpeedAction)
	
	rospy.loginfo("Waiting for joint_trajectory and joint_stiffness servers...")
	client.wait_for_server()
	stiffness_client.wait_for_server()
	angle_client.wait_for_server()
	rospy.loginfo("Done.")
	
	#inhibitWalkSrv()
	try:	
		goal = nao_msgs.msg.JointTrajectoryGoal()
		
		# move head: single joint, multiple keypoints
		goal.trajectory.joint_names = ["HeadYaw"]
		goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.0), positions = [1.0]))
		goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.0), positions = [-1.0]))
		goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(2.5), positions = [0.0]))
	
		
		rospy.loginfo("Sending goal...")
		client.send_goal(goal)
		client.wait_for_result()
		result = client.get_result()
		rospy.loginfo("Results: %s", str(result.goal_position.position))
		
		# crouching pose: single keypoint, multiple joints:
		goal.trajectory.joint_names = ["Body"]
		point = JointTrajectoryPoint()
		point.time_from_start = Duration(1.5)
		point.positions = [0.0,0.0, 					# head
			1.545, 0.33, -1.57, -0.486, 0.0, 0.0,		# left arm
			-0.3, 0.057, -0.744, 2.192, -1.122, -0.035, 	# left leg
			-0.3, 0.057, -0.744, 2.192, -1.122, -0.035,	# right leg
			1.545, -0.33, 1.57, 0.486, 0.0, 0.0]		# right arm
		goal.trajectory.points = [point]
	
		rospy.loginfo("Sending goal...")
		client.send_goal(goal)
		client.wait_for_result()
		rospy.loginfo("Getting results...")
		result = client.get_result()	
		print "Result:", ', '.join([str(n) for n in result.goal_position.position])
		
			
		# multiple joints, multiple keypoints:
		goal.trajectory.joint_names = ["HeadYaw", "HeadPitch"]
		goal.trajectory.points = []
		goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.5), 
														positions = [1.0, 1.0]))
		goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.0), 
														positions = [1.0, 0.0]))
		goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.5), 
														positions = [0.0, 0.0]))
	
		rospy.loginfo("Sending goal...")
		client.send_goal(goal)
		client.wait_for_result()
		rospy.loginfo("Getting results...")
		result = client.get_result()
		print "Result:", ', '.join([str(n) for n in result.goal_position.position])
		
		
		# multiple joints, single keypoint:
		goal.trajectory.joint_names = ["HeadYaw", "HeadPitch"]
		goal.trajectory.points = []
		goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.5), 
														positions = [0.5, 0.5]))
	
		rospy.loginfo("Sending goal...")
		client.send_goal(goal)
		client.wait_for_result()
		rospy.loginfo("Getting results...")
		result = client.get_result()
		print "Result:", ', '.join([str(n) for n in result.goal_position.position])
		
		
		angle_goal = nao_msgs.msg.JointAnglesWithSpeedGoal()
		angle_goal.joint_angles.relative = False
		angle_goal.joint_angles.joint_names = ["HeadYaw", "HeadPitch"]
		angle_goal.joint_angles.joint_angles = [1.0, 0.0]
		angle_goal.joint_angles.speed = 0.2
		angle_client.send_goal_and_wait(angle_goal)
		result = angle_client.get_result()
		rospy.loginfo("Angle results: %s", str(result.goal_position.position))
		
		stiffness_goal = nao_msgs.msg.JointTrajectoryGoal()
		stiffness_goal.trajectory.joint_names = ["Body"]
		stiffness_goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.5), positions = [1.0]))
		rospy.loginfo("Sending stiffness goal...")
		stiffness_client.send_goal(stiffness_goal)
		stiffness_client.wait_for_result()
		result = stiffness_client.get_result()
		rospy.loginfo("Stiffness results: %s", str(result.goal_position.position))
		
	finally:
		uninhibitWalkSrv()
	
	
	
if __name__ == '__main__':
	try:
		rospy.init_node('joint_angle_client')
		joint_angle_client()
		
	except rospy.ROSInterruptException:
  		print "program interrupted before completion"
  		