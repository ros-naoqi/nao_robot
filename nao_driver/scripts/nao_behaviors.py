#!/usr/bin/env python

#
# ROS node to control NAO's built-in and user-installed behaviors using NaoQI
# Tested with NaoQI: 1.12
#
# Copyright (c) 2012, Miguel Sarabia
# Imperial College London
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


import roslib

roslib.load_manifest('nao_driver')

import rospy
import actionlib

from nao_driver import *

from nao_msgs.msg import(
    RunBehaviorAction,
    RunBehaviorResult
    )

from nao_msgs.srv import (
    GetInstalledBehaviors, 
    GetInstalledBehaviorsResponse,
    )

class NaoBehaviors(NaoNode):
    #This should be treated as a constant
    NODE_NAME = "nao_behaviors"
    
    def __init__( self ):
        
        #Initialisation
        NaoNode.__init__( self )
        rospy.init_node( self.NODE_NAME )
        
        #We need this variable to be able to call stop behavior when preempted
        self.behavior = ""
        
        #Proxy for listingBehaviors and stopping them
        self.behaviorProxy = self.getProxy( "ALBehaviorManager" )
        
        # Register ROS services
        self.getInstalledBehaviorsService = rospy.Service(
            self.NODE_NAME + "/get_installed_behaviors",
            GetInstalledBehaviors,
            self.getInstalledBehaviors
            )
        
        #Prepare and start actionlib server
        self.actionlibServer = actionlib.SimpleActionServer(
            self.NODE_NAME + "/run_behavior",
            RunBehaviorAction,
            self.runBehavior,
            False
            )
        
        self.actionlibServer.register_preempt_callback( self.stopBehavior )
        
        self.actionlibServer.start()
    
    def getInstalledBehaviors( self, request ):
        result = self.behaviorProxy.getInstalledBehaviors()
        return GetInstalledBehaviorsResponse( result )
    
    
    def runBehavior( self, request ):
        #Note that this function is executed on a different thread
        
        #Save name of behavior to be run
        self.behavior = request.behavior
        result = RunBehaviorResult()
        result.noErrors = True
        
        if not self.behaviorProxy.isBehaviorInstalled( self.behavior ) :
            result.noErrors = False
            self.actionlibServer.set_aborted(
                result,
                "Unknown behavior: " + str(self.behavior) 
            )
            self.behavior = ""
            return
        
        #Execute behavior (risky if ALBehavior is not thread-safe!)
        self.behaviorProxy.runBehavior( self.behavior )
        
        # If we exited prematurely due to a call to stop behavior (which was
        # activated by a actionlib preemption signal) then set as preempted
        if self.actionlibServer.is_preempt_requested() :
            self.currentBehavior = ""
            self.actionlibServer.set_preempted()
            return
        
        #Everything went fine, finished normally
        self.behavior = ""
        self.actionlibServer.set_succeeded( result )
        return

    def stopBehavior( self ):
        if self.behavior != "" and self.actionlibServer.is_active() :
            self.behaviorProxy.stopBehavior( self.behavior )  


if __name__ == '__main__':
    node = NaoBehaviors()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
