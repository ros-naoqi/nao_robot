#!/usr/bin/env python

# SVN $HeadURL$
# SVN $Id$


#
# ROS node to read Nao's sensors and torso odometry through the Aldebaran API.
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2009 Armin Hornung, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


import roslib
roslib.load_manifest('nao_driver')
import rospy

from sensor_msgs.msg import JointState

from nao_msgs.msg import TorsoOdometry, TorsoIMU

from nao_driver import *

import threading
from threading import Thread

class NaoSensors(NaoNode, Thread):
    def __init__(self):
        NaoNode.__init__(self)
        Thread.__init__(self)

        # ROS initialization:
        rospy.init_node('nao_sensors')

        self.connectNaoQi()

        self.stopThread = False

        self.odomSleep = 1.0/rospy.get_param('~torso_odom_rate', 20.0)


        self.dataNamesList = ["DCM/Time",
                                "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value","Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value", "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value", "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value"]


        tf_prefix_param_name = rospy.search_param('tf_prefix')
        if tf_prefix_param_name:
            self.tf_prefix = rospy.get_param(tf_prefix_param_name)
        else:
            self.tf_prefix = ""

        self.base_frameID = rospy.get_param('~base_frame_id', "torso")
        if not(self.base_frameID[0] == '/'):
            self.base_frameID = self.tf_prefix + '/' + self.base_frameID

        # send cam odom?
        self.sendCamOdom = rospy.get_param('~send_cam_odom', False)
        # use sensor values or commanded (open-loop) values for joint angles
        self.useJointSensors = rospy.get_param('~use_joint_sensors', True) # (set to False in simulation!)
        # init. messages:
        self.torsoOdom = TorsoOdometry()
        self.camOdom = TorsoOdometry()
        self.torsoOdom.header.frame_id = rospy.get_param('~odom_frame_id', "odom")
        self.camOdom.header.frame_id = rospy.get_param('~odom_frame_id', "odom")
        if not(self.torsoOdom.header.frame_id[0] == '/'):
            self.torsoOdom.header.frame_id = self.tf_prefix + '/' + self.torsoOdom.header.frame_id
        if not(self.camOdom.header.frame_id[0] == '/'):
            self.camOdom.header.frame_id = self.tf_prefix + '/' + self.camOdom.header.frame_id
        self.torsoIMU = TorsoIMU()
        self.torsoIMU.header.frame_id = self.base_frameID
        self.jointState = JointState()
        self.jointState.name = self.motionProxy.getJointNames('Body')

        # simluated model misses some joints, we need to fill:
        if (len(self.jointState.name) == 22):
            self.jointState.name.insert(6,"LWristYaw")
            self.jointState.name.insert(7,"LHand")
            self.jointState.name.append("RWristYaw")
            self.jointState.name.append("RHand")

        msg = "Nao joints found: "+ str(self.jointState.name)
        rospy.logdebug(msg)


        if self.sendCamOdom:
            self.camOdomPub = rospy.Publisher("camera_odometry", TorsoOdometry)
        self.torsoOdomPub = rospy.Publisher("torso_odometry", TorsoOdometry)
        self.torsoIMUPub = rospy.Publisher("torso_imu", TorsoIMU)
        self.jointStatePub = rospy.Publisher("joint_states", JointState)

        rospy.loginfo("nao_sensors initialized")

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.getProxy("ALMotion")
        self.memProxy = self.getProxy("ALMemory")
        # TODO: check self.memProxy.version() for > 1.6
        if self.motionProxy is None or self.memProxy is None:
            exit(1)

    def run(self):
        """ Odometry thread code - collects and sends out odometry esimate. """
        while(not self.stopThread):
                #
                # Build odometry:
                #
            timestamp = rospy.Time.now()
            try:
                memData = self.memProxy.getListData(self.dataNamesList)
                 # odometry data:
                odomData = self.motionProxy.getPosition('Torso', motion.SPACE_WORLD, True)
                # camera data
                #camData = self.motionProxy.getTransform('CameraTop', motion.SPACE_WORLD, True)
                if self.sendCamOdom:
                    camData = self.motionProxy.getPosition('CameraTop', motion.SPACE_WORLD, True)
                positionData = self.motionProxy.getAngles('Body', self.useJointSensors)
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

            self.torsoOdom.header.stamp = timestamp
            self.camOdom.header.stamp = timestamp
            if len(odomData)==2:
                odomData = odomData[1]
            elif len(odomData)!=6:
                print "Error getting odom data"
                continue
            self.torsoOdom.x = odomData[0]
            self.torsoOdom.y = odomData[1]
            self.torsoOdom.z = odomData[2]
            self.torsoOdom.wx = odomData[3]
            self.torsoOdom.wy = odomData[4]
            self.torsoOdom.wz = odomData[5]

            if self.sendCamOdom:
                self.camOdom.x = camData[0]
                self.camOdom.y = camData[1]
                self.camOdom.z = camData[2]
                self.camOdom.wx = camData[3]
                self.camOdom.wy = camData[4]
                self.camOdom.wz = camData[5]

            self.torsoOdomPub.publish(self.torsoOdom)
            if self.sendCamOdom:
                self.camOdomPub.publish(self.camOdom)

            # Replace 'None' values with 0
            # (=> consistent behavior in 1.8 / 1.10 with 1.6)
            for i, m in enumerate(memData):
                if m is None:
                    memData[i] = 0

            if len(memData) != len(self.dataNamesList):
                print "memData length does not match expected length"
                print memData
                continue


            # IMU data:
            self.torsoIMU.header.stamp = timestamp
            self.torsoIMU.angleX = memData[1]
            self.torsoIMU.angleY = memData[2]
            self.torsoIMU.gyroX = memData[3]
            self.torsoIMU.gyroY = memData[4]
            self.torsoIMU.accelX = memData[5]
            self.torsoIMU.accelY = memData[6]
            self.torsoIMU.accelZ = memData[7]

            self.torsoIMUPub.publish(self.torsoIMU)


            #
            # Send JointState:
            #
            self.jointState.header.stamp = timestamp
            self.jointState.header.frame_id = self.base_frameID
            self.jointState.position = positionData

            # simulated model misses some joints, we need to fill:
            if (len(self.jointState.position) == 22):
                self.jointState.position.insert(6, 0.0)
                self.jointState.position.insert(7, 0.0)
                self.jointState.position.append(0.0)
                self.jointState.position.append(0.0)

            self.jointStatePub.publish(self.jointState)

            rospy.sleep(self.odomSleep)

if __name__ == '__main__':

    sensors = NaoSensors()
    sensors.start()

    rospy.spin()

    rospy.loginfo("Stopping nao_sensors ...")
    sensors.stopThread = True
    sensors.join()

    rospy.loginfo("nao_sensors stopped.")
    exit(0)
