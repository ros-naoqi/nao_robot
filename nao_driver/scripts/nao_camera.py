#! /usr/bin/env python

import roslib
roslib.load_manifest('nao_driver')
import rospy
from sensor_msgs.msg import Image, CameraInfo
from nao_driver import NaoNode
from sensor_msgs.srv import SetCameraInfo


from naoqi import ALProxy
from vision_definitions import *


class NaoCam (NaoNode):
    def __init__(self):
        NaoNode.__init__(self)
        rospy.init_node('nao_camera')
        self.camProxy = self.getProxy("ALVideoDevice")
        if self.camProxy is None:
            exit(1)

        # ROS pub/sub
        self.pub_img_ = rospy.Publisher('image_raw', Image)
        self.pub_info_ = rospy.Publisher('camera_info', CameraInfo)
        self.set_camera_info_service_ = rospy.Service('set_camera_info', SetCameraInfo, self.set_camera_info)
        # Messages
        self.info_ = CameraInfo()
        self.set_default_params_qvga(self.info_) #default params should be overwritten by service call
        # parameters
        self.camera_switch = rospy.get_param('~camera_switch', 0)
        if self.camera_switch == 0:
            self.frame_id = "/CameraTop_frame"
        elif self.camera_switch == 1:
            self.frame_id = "/CameraBottom_frame"
        else:
            rospy.logerr('Invalid camera_switch. Must be 0 or 1')
            exit(1)
        print "Using namespace ", rospy.get_namespace()
        print "using camera: ", self.camera_switch
        #TODO: parameters
        self.resolution = kQVGA
        self.colorSpace = kBGRColorSpace
        self.fps = 30
        # init
        self.nameId = ''
        self.subscribe()

    def subscribe(self):
        # unsubscribe for all zombie subscribers
        self.camProxy.unsubscribeAllInstances("rospy_gvm")

        # subscribe
        self.nameId = self.camProxy.subscribe("rospy_gvm", self.resolution, self.colorSpace, self.fps)
        #print "subscriber name is ", self.nameId

        # set params
        rospy.sleep(1)
        self.camProxy.setParam(kCameraSelectID, self.camera_switch)
        self.camProxy.setResolution(self.nameId, self.resolution)

    def set_camera_info(self, cameraInfoMsg):
        print "Received new camera info"
        self.info_ = cameraInfoMsg.camera_info



    def set_default_params_qvga(self, cam_info):
        cam_info.P[0] = 382.92
        cam_info.P[1] = 0.0
        cam_info.P[2] = 160.0
        cam_info.P[3] = 0
        cam_info.P[4] = 0.0
        cam_info.P[5] = 373.31
        cam_info.P[6] = 120.0
        cam_info.P[7] = 0.0
        cam_info.P[8] = 0.0
        cam_info.P[9] = 0.0
        cam_info.P[10] = 1.0
        cam_info.P[11] = 0.0

    def main_loop(self):
        img = Image()
        while not rospy.is_shutdown():
            #print "getting image..",
            image = self.camProxy.getImageRemote(self.nameId)
            #print "ok"
            # TODO: better time
            img.header.stamp = rospy.Time.now()
            img.header.frame_id = self.frame_id
            img.height = image[1]
            img.width = image[0]
            nbLayers = image[2]
            #colorspace = image[3]
            if image[3] == kYUVColorSpace:
                encoding = "mono8"
            elif image[3] == kRGBColorSpace:
                encoding = "rgb8"
            elif image[3] == kBGRColorSpace:
                encoding = "bgr8"
            else:
                rospy.logerror("Received unknown encoding: {0}".format(image[3]))

            img.encoding = encoding
            img.step = img.width * nbLayers
            img.data = image[6]
            self.info_.width = img.width
            self.info_.height = img.height
            self.info_.header = img.header
            self.pub_img_.publish(img)
            self.pub_info_.publish(self.info_)
            rospy.sleep(0.0001)# TODO: is this necessary?


        self.camProxy.unsubscribe(self.nameId)

if __name__ == "__main__":
    try:
        naocam = NaoCam()
        naocam.main_loop()
    except RuntimeError, e:
        rospy.logerr('Something went wrong: %s' % (e) )
    rospy.loginfo('Camera stopped')
