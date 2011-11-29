#!/usr/bin/env python

import math
import time
import naoqi
import motion
from naoqi import ALProxy
import roslib
roslib.load_manifest('nao_driver')
import rospy
send_robot_state=False
if send_robot_state:
    from robot_state.msg import RobotState
from scan_pose import scanPose
from scan_pose_new import scanPoseNew
from camera_walk_pose import cameraWalkPose

def head_scan(motionProxy,headPitchStart=26.0,headPitchEnd=-10.0,angularResolution=0.5,laserScanFreq=10,use_scan_pose=0):
    if use_scan_pose:
        scanPoseNew(motionProxy,math.pi*headPitchStart/180,0.0)

    if send_robot_state:
        robotStatePub = rospy.Publisher("robot_state", RobotState)
        rospy.sleep(0.25)
        robot_state = RobotState()
        robot_state.stamp = rospy.Time.now()
        robot_state.id = RobotState.SCANNING
        robotStatePub.publish(robot_state)
    # TODO: check limits of head and compare to headPitchEnd and headPitchStart
    headPitchStart = headPitchStart * math.pi/180.0
    headPitchEnd = headPitchEnd * math.pi/180.0
    degreePerSecond = angularResolution* laserScanFreq
    radiansPerSecond = degreePerSecond*math.pi/180.0
    #print("Radians per second: " + str(radiansPerSecond))
    #print("Degree per second: " + str(degreePerSecond))
    head_pitch_before_scan = motionProxy.getAngles("HeadPitch",True)[0]
    head_pitch_after_scan = head_pitch_before_scan
    #print("angles " + str(head_pitch_before_scan) + ", " + str(headPitchStart) + ", "  + str(headPitchEnd) + ",and " + str(head_pitch_after_scan))

    motionProxy.stiffnessInterpolation('Body',1.0, 0.5)
    t  = abs(head_pitch_before_scan - headPitchStart)/radiansPerSecond
    t2 = abs(headPitchStart - headPitchEnd)/radiansPerSecond
    t3 = abs(headPitchEnd - head_pitch_after_scan)/radiansPerSecond
    #print "ang dif is " + str(abs(head_pitch_before_scan - headPitchStart))
    #print "t is " + str(t)
    #exit(0)
    # set yaw to 0.0 and pitch to headPitchStart
    motionProxy.angleInterpolation('Head', [0.0, headPitchStart], t, True)
    motionProxy.angleInterpolation('Head', [0.0, headPitchEnd], t2, True)
    motionProxy.angleInterpolation('Head', [0.0, head_pitch_after_scan], t3, True)
    #ttsProxy.say("Scanning now")
    # list of angles and times
    #angleList = [headPitchStart, headPitchEnd, head_pitch_after_scan]
    #angularDifferences = [head_pitch_before_scan-headPitchStart,
    #                      headPitchEnd-headPitchStart,
    #                     head_pitch_before_scan-headPitchEnd]
                          #print(angularDifferences)
                          #timeList = [abs(s)/radiansPerSecond for s in angularDifferences]
    # need to accumulate time list (required by angularInterpolation)
    #accumulatedTime = 0.0
    #for i in range(0,len(timeList)):
        #    accumulatedTime = accumulatedTime + timeList[i]
        #timeList[i] = accumulatedTime
        #print(timeList)
        #motionProxy.angleInterpolation('HeadPitch', angleList, timeList, True)
    #ttsProxy.say("Done")
    if use_scan_pose:
        cameraWalkPose(motionProxy,math.pi*head_pitch_before_scan/180,0.0)

    if send_robot_state:
        rospy.sleep(1.0)
        robot_state = RobotState()
        robot_state.stamp = rospy.Time.now()
        robot_state.id = RobotState.SCAN_COMPLETE
        robotStatePub.publish(robot_state)

if __name__ == '__main__':

    from optparse import OptionParser
    rospy.init_node('headscan')

    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default="127.0.0.1",
                      help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
    parser.add_option("--pport", dest="pport", default=9559,
                      help="port of parent broker. Default is 9559.", metavar="PORT")

    parser.add_option("--s", dest="headpitchstart", default=26.0,
                      help="start head pitch", metavar="START")
    parser.add_option("--e", dest="headpitchend", default=-30.0,
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
        ttsProxy = ALProxy("ALTextToSpeech",options.pip,int(options.pport))
    except RuntimeError,e:    
        print("Error creating Proxy to motion, exiting...")
        print e
        exit(1)

    head_scan(motionProxy,headPitchStart,headPitchEnd,angularResolution,laserScanFreq,1)
    print("Scan completed...")
    exit(0)


