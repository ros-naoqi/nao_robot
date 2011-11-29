#!/usr/bin/env python

import math
import time
import naoqi
import motion
from naoqi import ALProxy
import roslib
roslib.load_manifest('nao_driver')
import rospy
use_robot_state_publisher = False
if use_robot_state_publisher:
    from robot_state.msg import RobotState

from scan_pose_bending import scanPoseBending
from camera_walk_pose import cameraWalkPose

def head_scan(motionProxy,headPitchStart=26.0,headPitchEnd=-10.0,angularResolution=0.5,laserScanFreq=10,use_scan_pose=False):
    start_time = rospy.Time.now()
    if(use_scan_pose):
        rospy.loginfo("HeadscanTop IS using scan_pose_new") 
    else:
        rospy.loginfo("HeadscanTop IS NOT using scan_pose_new") 

    if use_robot_state_publisher:
        robotStatePub = rospy.Publisher("robot_state", RobotState)
        rospy.sleep(0.5)
        robot_state = RobotState()

    # TODO: check limits of head and compare to headPitchEnd and headPitchStart
    headPitchStart = headPitchStart * math.pi/180.0
    headPitchEnd = headPitchEnd * math.pi/180.0
    headPitchInflectionPoint = headPitchEnd + (20.0*math.pi/180.0) # go further up than final scan pose
    degreePerSecond = angularResolution* laserScanFreq*0.33
    radiansPerSecond = degreePerSecond*math.pi/180.0
    #print("Radians per second: " + str(radiansPerSecond))
    #print("Degree per second: " + str(degreePerSecond))
    head_pitch_before_scan = motionProxy.getAngles("HeadPitch",True)[0]
    head_pitch_camera_learning = 22.0*math.pi/180.0
    if use_scan_pose:
        scanPoseBending(motionProxy,head_pitch_before_scan,0.0,2)

    #motionProxy.stiffnessInterpolation('Body',1.0, 0.5)
    # compute some times
    t  = abs(head_pitch_before_scan - headPitchStart)/radiansPerSecond
    t2 = abs(headPitchStart - headPitchEnd)/radiansPerSecond
    t3 = abs(headPitchEnd - head_pitch_camera_learning)/radiansPerSecond
    # set yaw to 0.0 and pitch to headPitchStart
    motionProxy.angleInterpolation('Head', [0.0, headPitchStart], t/4.0, True)
    
    if use_robot_state_publisher:
        robot_state.stamp = rospy.Time.now()
        robot_state.id = RobotState.SCANNING
        robotStatePub.publish(robot_state)

    # HACK to make nicer floor scans. TODO: improve me!
    angles = list()
    times = list()
    for i in range(0,9):
        angles.append( headPitchStart +  (i+1)*(headPitchEnd - headPitchStart)/10.0)
        tt = (i+1)*t2/10.0 
        times.append( tt*tt/t2 )

    rospy.logdebug("using times %s"%(str(times)))
    rospy.logdebug("using angles %s"%(str(angles)))
    motionProxy.angleInterpolation('HeadPitch', angles, times, True)
    #motionProxy.angleInterpolation('Head', [0.0, headPitchEnd], t2, True)
    if use_robot_state_publisher:
        rospy.sleep(0.5)
        robot_state = RobotState()
        robot_state.stamp = rospy.Time.now()
        robot_state.id = RobotState.SCAN_COMPLETE
        rospy.sleep(0.5)

    # adopt final pose  TODO: this should not necessary be cameraWalkPose
    if use_scan_pose:
        cameraWalkPose(motionProxy,head_pitch_camera_learning,0.0)
    #else:
	#t4 = abs(head_pitch_camera_learning - head_pitch_after_scan)/radiansPerSecond
	#head_pitch_after_scan = head_pitch_before_scan
	#head_pitch_after_scan = 20.0*math.pi/180.0
        #motionProxy.angleInterpolation('Head', [0.0, head_pitch_after_scan], t4, True)
    if use_robot_state_publisher:
        # wait to trigger point cloud assembler and image classifier
        rospy.sleep(0.5)
        robotStatePub.publish(robot_state)

    dur = rospy.Time.now() - start_time
    rospy.loginfo("Scan took %f seconds."%(dur.to_sec()))

if __name__ == '__main__':

    from optparse import OptionParser
    rospy.init_node('headscan')

    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default="127.0.0.1",
                      help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
    parser.add_option("--pport", dest="pport", default=9559,
                      help="port of parent broker. Default is 9559.", metavar="PORT")

    parser.add_option("--s", dest="headpitchstart", default=27.0,
                      help="start head pitch", metavar="START")
    parser.add_option("--e", dest="headpitchend", default=-10.0,
                      help="end head pitch (negative angles are towards sky)",
                      metavar="END")
    parser.add_option("--speed", dest="headpitchspeed", default=10,
                      help="head pitch speed(degree per second)", metavar="SPEED")

    (options, args) = parser.parse_args()

    headPitchStart = float(options.headpitchstart )
    headPitchEnd = float(options.headpitchend)

    # TODO: useless..
    pitch_speed = float(options.headpitchspeed)
    angularResolution = 1
    laserScanFreq = pitch_speed
    try:
        motionProxy = ALProxy("ALMotion",options.pip,int(options.pport))
    except RuntimeError,e:    
        print("Error creating Proxy to motion, exiting...")
        print e
        exit(1)

    try:
	ttsProxy = ALProxy("ALTextToSpeech",options.pip,int(options.pport))
    except RuntimeError,e:    
        print("Error creating Proxy to tts ...")
        print e


    head_scan(motionProxy,headPitchStart,headPitchEnd,angularResolution,laserScanFreq,False)
    print("Scan completed...")
    exit(0)


