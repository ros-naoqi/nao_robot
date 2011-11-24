import rospy

# go to crouching position
def initPose(motionProxy):
	numJoints = len(motionProxy.getJointNames('Body'))
	
	allAngles = [0.0,0.0, 					# head
		1.39, 0.34, -1.39, -1.04, 0.0, 0.0,		# left arm
		0.0, 0.0, -0.43, 0.69, -0.34, 0.0, 	# left leg
		0.0, 0.0, -0.43, 0.69, -0.34, 0.0,	# right leg
		1.39, -0.34, 1.39, 1.04, 0.0, 0.0]		# right arm
	
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
