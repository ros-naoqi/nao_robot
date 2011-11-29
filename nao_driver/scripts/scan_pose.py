#!/usr/bin/env python

import math

# TODO: are these all required? Shouldn't be for a simple pose...
import naoqi
import motion
from naoqi import ALProxy


import roslib
roslib.load_manifest('nao_driver')
import rospy


# go to crouching position
def scanPose(motionProxy,head_pitch,head_yaw):
	numJoints = len(motionProxy.getJointNames('Body'))
	print "received " + str(head_pitch) + " " + str(head_yaw)
	allAngles = [head_yaw,head_pitch, 				# head
		1.574, 0.212, -0.011, -0.003, 0.003, 0.01,	# left arm
		-0.423, -0.023, -0.861, 2.223, -1.174, 0.023, 	# left leg
		-0.442, 0.089, -0.823, 2.185, -1.162, -0.066,	# right leg
		1.574, -0.212, -0.011, -0.005, 0.003, -0.01]		# right arm

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

if __name__ == '__main__':
    from optparse import OptionParser
    rospy.init_node('headscan')

    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default="127.0.0.1",
                      help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
    parser.add_option("--pport", dest="pport", default=9559,
                      help="port of parent broker. Default is 9559.", metavar="PORT")

    (options, args) = parser.parse_args()

    try:
        motionProxy = ALProxy("ALMotion",options.pip,int(options.pport))
        ttsProxy = ALProxy("ALTextToSpeech",options.pip,int(options.pport))
    except RuntimeError,e:    
        print("Error creating Proxy to motion, exiting...")
        print e
        exit(1)

    scan_pose(motionProxy,-26.0*math.pi/180.0,0.0)
    print("Scan completed...")
    exit(0)

