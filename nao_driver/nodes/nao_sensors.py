#!/usr/bin/env python

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


import rospy

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from nao_driver import NaoNode

from tf import transformations
import tf

# NAOqi specific
import motion

class NaoSensors(NaoNode):
    def __init__(self):
        NaoNode.__init__(self, 'nao_sensors')

        self.connectNaoQi()

        # default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
        self.sensorRate = rospy.Rate(rospy.get_param('~sensor_rate', 25.0))

        self.dataNamesList = ["DCM/Time",
                                "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value","Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value", "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value", "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value"]


        tf_prefix_param_name = rospy.search_param('tf_prefix')
        if tf_prefix_param_name:
            self.tf_prefix = rospy.get_param(tf_prefix_param_name)
        else:
            self.tf_prefix = ""
        
        # To stop odometry tf being broadcast
        self.broadcast_odometry = rospy.get_param('~broadcast_odometry', True)

        self.base_frameID = rospy.get_param('~base_frame_id', "base_link")
        if not(self.base_frameID[0] == '/'):
            self.base_frameID = self.tf_prefix + '/' + self.base_frameID

        # use sensor values or commanded (open-loop) values for joint angles
        self.useJointSensors = rospy.get_param('~use_joint_sensors', True) # (set to False in simulation!)
        # init. messages:
        self.torsoOdom = Odometry()
        self.torsoOdom.header.frame_id = rospy.get_param('~odom_frame_id', "odom")
        if not(self.torsoOdom.header.frame_id[0] == '/'):
            self.torsoOdom.header.frame_id = self.tf_prefix + '/' + self.torsoOdom.header.frame_id

        self.torsoIMU = Imu()
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

        self.torsoOdomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.torsoIMUPub = rospy.Publisher("imu", Imu, queue_size=10)
        self.jointStatePub = rospy.Publisher("joint_states", JointState, queue_size=10)

        self.tf_br = tf.TransformBroadcaster()

        rospy.loginfo("nao_sensors initialized")

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.motionProxy = self.get_proxy("ALMotion")
        self.memProxy = self.get_proxy("ALMemory")
        if self.motionProxy is None or self.memProxy is None:
            exit(1)

    def run(self):
        """ Odometry thread code - collects and sends out odometry esimate. """
        while self.is_looping():
                #
                # Build odometry:
                #
            timestamp = rospy.Time.now()
            try:
                memData = self.memProxy.getListData(self.dataNamesList)
                 # odometry data:
                odomData = self.motionProxy.getPosition('Torso', motion.SPACE_WORLD, True)
                positionData = self.motionProxy.getAngles('Body', self.useJointSensors)
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

            self.torsoOdom.header.stamp = timestamp
            if len(odomData)==2:
                odomData = odomData[1]
            elif len(odomData)!=6:
                print "Error getting odom data"
                continue
            
            self.torsoOdom.pose.pose.position.x = odomData[0]
            self.torsoOdom.pose.pose.position.y = odomData[1]
            self.torsoOdom.pose.pose.position.z = odomData[2]
            q = transformations.quaternion_from_euler(odomData[3], odomData[4], odomData[5])
            self.torsoOdom.pose.pose.orientation.x = q[0]
            self.torsoOdom.pose.pose.orientation.y = q[1]
            self.torsoOdom.pose.pose.orientation.z = q[2]
            self.torsoOdom.pose.pose.orientation.w = q[3]

            t = self.torsoOdom.pose.pose.position
            q = self.torsoOdom.pose.pose.orientation
            
            if self.broadcast_odometry:
                self.tf_br.sendTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w),
                                         timestamp, self.base_frameID, self.torsoOdom.header.frame_id)

            self.torsoOdomPub.publish(self.torsoOdom)

            # Replace 'None' values with 0
            # (=> consistent behavior in 1.8 / 1.10 with 1.6)
            # TODO: still required with 1.12 / 1.14?
            for i, m in enumerate(memData):
                if m is None:
                    memData[i] = 0

            if len(memData) != len(self.dataNamesList):
                print "memData length does not match expected length"
                print memData
                continue


            # IMU data:
            self.torsoIMU.header.stamp = timestamp
            q = transformations.quaternion_from_euler(memData[1], memData[2], memData[3])
            self.torsoIMU.orientation.x = q[0]
            self.torsoIMU.orientation.y = q[1]
            self.torsoIMU.orientation.z = q[2]
            self.torsoIMU.orientation.w = q[3]

            self.torsoIMU.angular_velocity.x = memData[4]
            self.torsoIMU.angular_velocity.y = memData[5]
            self.torsoIMU.angular_velocity.z = memData[6] # currently always 0

            self.torsoIMU.linear_acceleration.x = memData[7]
            self.torsoIMU.linear_acceleration.y = memData[8]
            self.torsoIMU.linear_acceleration.z = memData[9]

            # covariances unknown
            # cf http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html
            self.torsoIMU.orientation_covariance[0] = -1
            self.torsoIMU.angular_velocity_covariance[0] = -1
            self.torsoIMU.linear_acceleration_covariance[0] = -1

            self.torsoIMUPub.publish(self.torsoIMU)

            # Send JointState:
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

            self.sensorRate.sleep()

if __name__ == '__main__':

    sensors = NaoSensors()
    sensors.start()

    rospy.spin()
    exit(0)
