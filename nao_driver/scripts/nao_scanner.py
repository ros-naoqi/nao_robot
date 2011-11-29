#!/usr/bin/env python

import roslib
roslib.load_manifest('nao_driver')
import rospy

import math
import time
from math import fabs

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from nao_msgs.msg import MotionCommandBtn

import naoqi
import motion
from naoqi import ALProxy

from nao_walker import NaoWalker
from crouch import crouch
from init_pose import initPose
from scan_pose import scanPose
from camera_walk_pose import cameraWalkPose
from launch_simulation.msg import robot_state


class NaoScanner(NaoWalker):
    def __init__(self, ip, port, head_pitch_start, head_pitch_end, pitch_speed,
                head_pitch_after_scan):
        NaoWalker.__init__(self,ip,port)
        self.scanning=False;
        self.head_pitch_start = head_pitch_start
        self.head_pitch_end = head_pitch_end
        self.head_pitch_after_scan = head_pitch_after_scan
        self.head_yaw = 0.0
        self.pitch_speed = pitch_speed
        self.RobotStatePub = rospy.Publisher("nao_state", robot_state)

    def handleMotionBtn(self,data):
        print "Handling Button..."
        if (data.button == MotionCommandBtn.crouchNoStiff):
            self.stopWalk()
            crouch(self.motionProxy)
            self.motionProxy.stiffnessInterpolation('Body',0.0, 0.5)
            self.say("Stiffness removed")
        elif (data.button == MotionCommandBtn.initPose):
                initPose(self.motionProxy)
        elif (data.button == MotionCommandBtn.stiffnessOn):
                self.motionProxy.stiffnessInterpolation('Body',1.0, 0.5)
        elif (data.button == MotionCommandBtn.stiffnessOff):
                self.motionProxy.stiffnessInterpolation('Body',0.0, 0.5)
        elif (data.button == MotionCommandBtn.startScan):
                self.startScan()
        elif (data.button == MotionCommandBtn.stopScan):
                self.stopScan()

    def handleMotion(self,data):
        if self.scanning != True:
            NaoWalker.handleMotion(self,data)
        else:
            print "Not handling motion because scanning is active"

    def startScan(self):
        self.scanning=True
        print "Scanning started ..."
        self.stopWalk()
        # send rs message
        rs = robot_state()
        rs.data = "scanning"
        rs.stamp = rospy.Time.now()
        self.RobotStatePub.publish(rs)
        # call actual scan function
        head_pitch = self.head_pitch_start
        head_yaw = self.head_yaw
        scanPose(self.motionProxy,head_pitch,head_yaw)
        self.run_scan()
        # get into final pose
        #cameraWalkPose(self.motionProxy, self.head_pitch_after_scan, head_yaw)
        # send rs message
        rs = robot_state()
        rs.data = "scan_complete"
        rs.stamp = rospy.Time.now()
        self.RobotStatePub.publish(rs)
        # inform user everything went fine
        st_say = String()
        st_say.data = "Scan completed"
        self.handleSpeech(st_say)
        self.scanning = False # re-enable handleMotion
        print "..Scan completed"
        

    def stopScan(self):
        if( self.scanning != True):
            print "... Scanning already stopped. Nothing to do..."
            return
        #else
        self.scanning = False
        print "... Scanning stopped"

    def run_scan(self):
        use_changeAngles = False
        if use_changeAngles:
            pitchChange = self.head_pitch_start - self.head_pitch_end
            pitchSpeed  = self.pitch_speed
            try:
                read_angles = self.motionProxy.getAngles("HeadPitch",True)
                print "I read " + str(read_angles)
                print self.motionProxy.getLimits("HeadPitch")
                self.motionProxy.setAngles(["HeadPitch"], [-0.3], 0.2)
                #self.motionProxy.setAngles("HeadPitch", -0.2, 0.1)
                #self.motionProxy.changeAngles("HeadPitch", 1.0, 0.2)
            except RuntimeError,e:
                rospy.logerr("Exception caught:\n%s", e)

        else: # old method, setting increments manually
            read_pitch = self.motionProxy.getAngles("HeadPitch", True)
            while abs(read_pitch[0] - self.head_pitch_end) > 0.05:
                self.motionProxy.setAngles("HeadPitch", self.head_pitch_end, self.pitch_speed)
                read_pitch = self.motionProxy.getAngles("HeadPitch", True)




            #self.stopScan()



if __name__ == '__main__':

    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default="127.0.0.1",
                      help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
    parser.add_option("--pport", dest="pport", default=9559,
                      help="port of parent broker. Default is 9559.", metavar="PORT")

    (options, args) = parser.parse_args()

    head_pitch_start = 26.0 * math.pi/180.0
    head_pitch_end = -5.0 * math.pi/180.0
    pitch_speed = 0.1
    head_pitch_after_scan = 20 * math.pi/180.0
    scanner = NaoScanner(options.pip,
                         int(options.pport),head_pitch_start,head_pitch_end,
                         pitch_speed, head_pitch_after_scan)
    rospy.spin()
    rospy.loginfo("Stopping NaoScanner...")
    scanner.stopWalk()

    rospy.loginfo("NaoScanner stopped.")
    exit(0)
