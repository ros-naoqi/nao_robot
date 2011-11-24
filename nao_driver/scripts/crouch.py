import rospy

# go to crouching position
def crouch(motionProxy):
	numJoints = len(motionProxy.getJointNames('Body'))
	
	allAngles = [0.0,0.0, 					# head
		1.545, 0.33, -1.57, -0.486, 0.0, 0.0,		# left arm
		-0.3, 0.057, -0.744, 2.192, -1.122, -0.035, 	# left leg
		-0.3, 0.057, -0.744, 2.192, -1.122, -0.035,	# right leg
		1.545, -0.33, 1.57, 0.486, 0.0, 0.0]		# right arm
	
	if (numJoints == 26):
		angles = allAngles
	elif (numJoints == 22):  # no hands (e.g. simulator)
		angles = allAngles[0:6] + allAngles[8:24]
	else:
	   rospy.logerr("Unkown number of joints: %d", numJoints)
	   return 
		
	try:
		motionProxy.angleInterpolation('Body', angles, 1.5, True);

	except RuntimeError,e:
		print "An error has been caught"
		print e
