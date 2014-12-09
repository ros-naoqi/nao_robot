#!/usr/bin/env python

#
# ROS node to interface with Naoqi Autonomous Life modules
# Tested with NaoQI: 2.1
#
# Copyright (c) 2014, Kei Okada
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the Imperial College London nor the names of its
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
import distutils

from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALBroker, ALProxy, ALModule)

from std_srvs.srv import( Empty, EmptyResponse )

class NaoALife(NaoqiNode):
    #This should be treated as a constant
    NODE_NAME = "nao_alife"

    def __init__( self ):

        #Initialisation
        NaoqiNode.__init__( self, self.NODE_NAME )
        if self.get_version() < distutils.version.LooseVersion('2.0'):
            rospy.logwarn("{} is only available on NaoQi version 2.0 or higher, your version is {}".format(self.NODE_NAME, self.get_version()))
            exit(1)

        #Proxy to interface with LEDs
        self.proxy = self.get_proxy( "ALAutonomousLife" )

        # Register ROS services
        self.disabled_srv = rospy.Service("~disabled", Empty, self.disabled )
        self.solitary_srv = rospy.Service("~solitary", Empty, self.solitary )
        self.interactive_srv = rospy.Service("~interactive", Empty, self.interactive )
        self.safeguard_srv = rospy.Service("~safeguard", Empty, self.safeguard )

    def setstate( self, state):
        try:
            self.proxy.setState( state )
        except Exception as e:
            rospy.logwarn("Could not transit from " + self.proxy.getState() + " to " + state)
            rospy.logerr(e)
        return EmptyResponse()
        
    def disabled( self, request = None ):
        return self.setstate('disabled')
    def solitary( self, request = None ):
        return self.setstate('solitary')
    def interactive( self, request = None ):
        return self.setstate('interactive')
    def safeguard( self, request = None ):
        return self.setstate('safeguard')

if __name__ == '__main__':
    node = NaoALife()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
