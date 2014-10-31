#!/usr/bin/env python

#
# ROS node to read Nao's bumpers and tactile sensors
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2010 Stefan Osswald, University of Freiburg
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

import naoqi
from nao_msgs.msg import TactileTouch, Bumper
from std_msgs.msg import Bool
from nao_driver import NaoNode
from naoqi import ( ALModule, ALBroker, ALProxy )

#
# Notes:
# - A port number > 0 for the module must be specified.
#   If port 0 is used, a free port will be assigned automatically,
#   but naoqi is unable to pick up the assigned port number, leaving
#   the module unable to communicate with naoqi (1.10.52).
# 
# - Callback functions _must_ have a docstring, otherwise they won't get bound.
# 
# - Shutting down the broker manually will result in a deadlock,
#   not shutting down the broker will sometimes result in an exception 
#   when the script ends (1.10.52).
#

class NaoTactile(ALModule):
    "Sends callbacks for tactile touch, bumper press and foot contact to ROS"
    def __init__(self, moduleName):
        # get connection from command line:
        from optparse import OptionParser

        parser = OptionParser()
        parser.add_option("--ip", dest="ip", default="",
                          help="IP/hostname of broker. Default is system's default IP address.", metavar="IP")
        parser.add_option("--port", dest="port", default=0,
                          help="IP/hostname of broker. Default is automatic port.", metavar="PORT")
        parser.add_option("--pip", dest="pip", default="127.0.0.1",
                          help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
        parser.add_option("--pport", dest="pport", default=9559,
                          help="port of parent broker. Default is 9559.", metavar="PORT")

        (options, args) = parser.parse_args()
        self.ip = options.ip
        self.port = int(options.port)
        self.pip = options.pip
        self.pport = int(options.pport)
        self.moduleName = moduleName
        
        self.init_almodule()
        
        # ROS initialization:
        rospy.init_node('nao_tactile')
             
        # init. messages:
        self.tactile = TactileTouch()              
        self.bumper = Bumper()
        self.tactilePub = rospy.Publisher("tactile_touch", TactileTouch, queue_size=10)
        self.bumperPub = rospy.Publisher("bumper", Bumper, queue_size=10)
        
        try:
            footContact = self.memProxy.getData("footContact", 0)
        except RuntimeError:
            footContact = None
            
        if footContact is None:
            self.hasFootContactKey = False
            rospy.loginfo("Foot contact key is not present in ALMemory, will not publish to foot_contact topic.")
        else:
            self.hasFootContactKey = True
            self.footContactPub = rospy.Publisher("foot_contact", Bool, latch=True, queue_size=10)
            self.footContactPub.publish(footContact > 0.0)

        # constants in TactileTouch and Bumper will not be available in callback functions
        # as they are executed in the parent broker context (i.e. on robot),
        # so they have to be copied to member variables
        self.tactileTouchFrontButton = TactileTouch.buttonFront;
        self.tactileTouchMiddleButton = TactileTouch.buttonMiddle;
        self.tactileTouchRearButton = TactileTouch.buttonRear;
        self.bumperRightButton = Bumper.right;
        self.bumperLeftButton = Bumper.left;
        
        self.subscribe()
        
        rospy.loginfo("nao_tactile initialized")

    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)
        
        self.memProxy = ALProxy("ALMemory",self.pip,self.pport)
        # TODO: check self.memProxy.version() for > 1.6
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)


    def shutdown(self): 
        self.unsubscribe()
        # Shutting down broker seems to be not necessary any more
        # try:
        #     self.broker.shutdown()
        # except RuntimeError,e:
        #     rospy.logwarn("Could not shut down Python Broker: %s", e)


    def subscribe(self):
        self.memProxy.subscribeToEvent("FrontTactilTouched", self.moduleName, "onTactileChanged")
        self.memProxy.subscribeToEvent("MiddleTactilTouched", self.moduleName, "onTactileChanged")
        self.memProxy.subscribeToEvent("RearTactilTouched", self.moduleName, "onTactileChanged")
        self.memProxy.subscribeToEvent("RightBumperPressed", self.moduleName, "onBumperChanged")
        self.memProxy.subscribeToEvent("LeftBumperPressed", self.moduleName, "onBumperChanged")
        if self.hasFootContactKey:
            self.memProxy.subscribeToEvent("footContactChanged", self.moduleName, "onFootContactChanged")


    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent("FrontTactilTouched", self.moduleName)
        self.memProxy.unsubscribeToEvent("MiddleTactilTouched", self.moduleName)
        self.memProxy.unsubscribeToEvent("RearTactilTouched", self.moduleName)
        self.memProxy.unsubscribeToEvent("RightBumperPressed", self.moduleName)
        self.memProxy.unsubscribeToEvent("LeftBumperPressed", self.moduleName)
        if self.hasFootContactKey:
            self.memProxy.unsubscribeToEvent("footContactChanged", self.moduleName)


    def onTactileChanged(self, strVarName, value, strMessage):
        "Called when tactile touch status changes in ALMemory"
        if strVarName == "FrontTactilTouched":
            self.tactile.button = self.tactileTouchFrontButton
        elif strVarName == "MiddleTactilTouched":
            self.tactile.button = self.tactileTouchMiddleButton
        elif strVarName == "RearTactilTouched":
            self.tactile.button = self.tactileTouchRearButton

        self.tactile.state = int(value);
        self.tactilePub.publish(self.tactile)
        rospy.logdebug("tactile touched: name=%s, value=%d, message=%s.", strVarName, value, strMessage);

    def onBumperChanged(self, strVarName, value, strMessage):
        "Called when bumper status changes in ALMemory"
        if strVarName == "RightBumperPressed":
            self.bumper.bumper = self.bumperRightButton
        elif strVarName == "LeftBumperPressed":
            self.bumper.bumper = self.bumperLeftButton

        self.bumper.state = int(value);
        self.bumperPub.publish(self.bumper)
        rospy.logdebug("bumper pressed: name=%s, value=%d, message=%s.", strVarName, value, strMessage);
        
    def onFootContactChanged(self, strVarName, value, strMessage):
        "Called when foot contact changes in ALMemory"
        self.footContactPub.publish(value > 0.0)

if __name__ == '__main__':
    ROSNaoTactileModule = NaoTactile("ROSNaoTactileModule")

    rospy.spin()
    
    rospy.loginfo("Stopping nao_tactile ...")
    ROSNaoTactileModule.shutdown();        
    rospy.loginfo("nao_tactile stopped.")
    exit(0)
