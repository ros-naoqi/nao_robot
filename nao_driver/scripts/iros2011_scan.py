#!/usr/bin/env python

import math
import time
import naoqi
import motion
from naoqi import ALProxy
import roslib
roslib.load_manifest('nao_driver')
import rospy

from init_pose import initPose
from scan_pose_bending import scanPoseBending
from camera_walk_pose import cameraWalkPose
from std_msgs.msg import Bool

def head_scan(motionProxy,ttsProxy,headPitchStart=26.0,headPitchEnd=-10.0,angularResolution=0.5,laserScanFreq=10,use_scan_pose=False):
    scanInfoPub = rospy.Publisher("scan_info", Bool)
    scanInfo = Bool()
    
    # TODO: check limits of head and compare to headPitchEnd and headPitchStart
    headPitchStart = headPitchStart * math.pi/180.0
    headPitchEnd = headPitchEnd * math.pi/180.0
    degreePerSecond = angularResolution* laserScanFreq*0.33
    radiansPerSecond = degreePerSecond*math.pi/180.0
    radiansPerSecondFast = 0.35
    #print("Radians per second: " + str(radiansPerSecond))
    #print("Degree per second: " + str(degreePerSecond))
    head_pitch_before_scan = motionProxy.getAngles("HeadPitch",True)[0]
    head_pitch_after_scan = head_pitch_before_scan
    if use_scan_pose:
        initPose(motionProxy)
        scanPoseBending(motionProxy,head_pitch_before_scan,0.0,2)

    # compute some times
    t1  = abs(head_pitch_before_scan - headPitchStart)/radiansPerSecondFast
    t2 = abs(headPitchStart - headPitchEnd)/radiansPerSecond
    t3 = abs(headPitchEnd - head_pitch_after_scan)/radiansPerSecondFast

    motionProxy.angleInterpolation('HeadPitch', [headPitchStart], [t1], True)
    rospy.sleep(0.5)
    if (not ttsProxy is None):
        try:
            ttsid = ttsProxy.post.say("Start scan")
            #ttsProxy.wait(ttsid, 0)
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
    startTime = rospy.Time.now().to_sec();    
    scanInfo.data = True
    scanInfoPub.publish(scanInfo)

    motionProxy.angleInterpolation('HeadPitch', [headPitchEnd], [t2], True)
    
    stopTime = rospy.Time.now().to_sec();
    scanInfo.data = False
    scanInfoPub.publish(scanInfo)
    
    if (not ttsProxy is None):
        try:
            ttsid = ttsProxy.post.say("Scan complete.")
            #ttsProxy.wait(ttsid, 0)
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
    rospy.sleep(0.5)
    

    # adopt final pose  TODO: this should not necessary be cameraWalkPose
    if use_scan_pose:
        cameraWalkPose(motionProxy, head_pitch_after_scan, 0.0)

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
    parser.add_option("--pose", dest="usescanpose", default=False, help="use scan pose", metavar="BOOL")

    (options, args) = parser.parse_args()

    headPitchStart = float(options.headpitchstart )
    headPitchEnd = float(options.headpitchend)

    # TODO: useless..
    pitch_speed = float(options.headpitchspeed)
    useScanPose = bool(options.usescanpose)
    angularResolution = 1
    laserScanFreq = pitch_speed
    try:
        motionProxy = ALProxy("ALMotion",options.pip,int(options.pport))
        ttsProxy = ALProxy("ALTextToSpeech",options.pip,int(options.pport))
    except RuntimeError,e:    
        print("Error creating Proxy to motion, exiting...")
        print e
        exit(1)

    head_scan(motionProxy,headPitchStart,headPitchEnd,angularResolution,laserScanFreq,useScanPose)
    print("Scan completed...")
    exit(0)


