#!/usr/bin/env python

import roslib
roslib.load_manifest('nao_driver')
import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo


def fill_set_camera_info():
    cam_info = CameraInfo()
    cam_info.header.frame_id = '/CameraTop_frame'
    cam_info.header.stamp = rospy.Time.now()
    cam_info.P[0] = 640.0
    cam_info.P[1] = 0.0
    cam_info.P[2] = 320.0
    cam_info.P[3] = 0
    cam_info.P[4] = 0.0
    cam_info.P[5] = 373.31
    cam_info.P[6] = 120.0
    cam_info.P[7] = 0.0
    cam_info.P[8] = 0.0
    cam_info.P[9] = 0.0
    cam_info.P[10] = 1.0
    cam_info.P[11] = 0.0
    setCameraInfo = SetCameraInfo()
    setCameraInfo.camera_info = cam_info
    return setCameraInfo


def call_service():
    setCameraInfo = fill_set_camera_info()
    rospy.wait_for_service('set_camera_info')
    try:
        set_camera_info = rospy.ServiceProxy('set_camera_info', SetCameraInfo)
        print "proxy ready"
        response = set_camera_info(setCameraInfo.camera_info)
        print response.status_message
        return response.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    rospy.init_node('set_camera_info')
    ret = call_service()
    print "Return status is: ", ret
    exit(ret)
