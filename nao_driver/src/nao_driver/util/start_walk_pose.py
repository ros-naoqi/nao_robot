import rospy

def startWalkPose(motionProxy):
	numJoints = len(motionProxy.getJointNames('Body'))
	
	allAngles = [
         0.0076280385255813599,     # HeadPitch
         0.019900038838386536,      # HeadYaw
         1.8085440397262573,        # LAnklePitch
         0.21165004372596741,       # LAnkleRoll
        -1.4680800437927246,        # LElbowRoll
        -0.52918803691864014,       # LElbowYaw
         0.0091620385646820068,     # LHand
         0.0011173889506608248,     # LHipPitch
         0.001575961709022522,      # LHipRoll
         0.024585962295532227,      # LHipYawPitch
        -0.37885606288909912,       # LKneePitch
         0.93723207712173462,       # LShoulderPitch
        -0.56301999092102051,       # LShoulderRoll
        -0.019900038838386536,      # LWristYaw
         0.001575961709022522,      # RAnklePitch
         0.024585962295532227,      # RAnkleRoll
        -0.37893998622894287,       # RElbowRoll
         0.9419180154800415,        # RElbowYaw
        -0.55986803770065308,       # RHand
        -0.024502038955688477,      # RHipPitch
         1.8224339485168457,        # RHipRoll
        -0.22247196733951569,       # RHipYawPitch
         1.4756660461425781,        # RKneePitch
         0.52160197496414185,       # RShoulderPitch
         0.001492038369178772,      # RShoulderRoll
         0.025117311626672745]      # RWristYaw
	
	if (numJoints == 26):
		angles = allAngles
	elif (numJoints == 22):  #  no hands (e.g. simulator)
		angles = allAngles[0:6] + allAngles[8:24]
	else:
	   rospy.logerr("Unkown number of joints: %d", numJoints)
	   return 
		
	try:
		motionProxy.angleInterpolation('Body', angles, 1.5, True);

	except RuntimeError,e:
		print "An error has been caught"
		print e
