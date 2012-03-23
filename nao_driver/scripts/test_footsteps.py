#!/usr/bin/env python

import roslib
roslib.load_manifest('nao_driver')
import rospy
from rospy.exceptions import ROSException


import roslib.rostime
from roslib.rostime import Duration


import std_srvs
from std_srvs.srv import Empty
import humanoid_nav_msgs
from humanoid_nav_msgs.msg import StepTarget
from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceRequest

def footstep_client():
	inhibitWalkSrv = rospy.ServiceProxy("inhibit_walk", std_srvs.srv.Empty)
	uninhibitWalkSrv = rospy.ServiceProxy("uninhibit_walk", std_srvs.srv.Empty)		
	

	
	rospy.loginfo("Waiting for footstep server...")
	rospy.wait_for_service('footstep_srv')
	client = rospy.ServiceProxy("footstep_srv", humanoid_nav_msgs.srv.StepTargetService)
	rospy.loginfo("Done.")
	
#	try:
	goal = StepTarget()
	goal.leg = 0
	goal.pose.x = 0.08
	goal.pose.theta = 0.3
	client(goal)
	
	goal.leg = 1
	goal.pose.theta = 0.3
	client(goal)
	
	goal.leg = 0
	goal.pose.x = 0.0
	goal.pose.y = -0.16
	client(goal)
	
	goal.leg = 1
	goal.pose.x = 0.00
	goal.pose.y = 0.088
	goal.pose.theta = 0.0
	client(goal)
	
	goal.leg = 0
	goal.pose.x = 0.0
	goal.pose.y = -0.16
	client(goal)
	
	goal.leg = 1
	goal.pose.x = 0.00
	goal.pose.y = 0.0
	goal.pose.theta = 0.0
	client(goal)
	
	
	
if __name__ == '__main__':
	try:
		rospy.init_node('footstep_test_client')
		footstep_client()
		
	except rospy.ROSInterruptException:
  		print "program interrupted before completion"
  		