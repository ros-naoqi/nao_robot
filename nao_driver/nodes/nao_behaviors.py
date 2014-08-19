#!/usr/bin/env python

#
# ROS node to control NAO's built-in and user-installed behaviors using NaoQI
# Tested with NaoQI: 1.12
#
# Copyright (c) 2012, 2013 Miguel Sarabia
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

import threading
import rospy
import actionlib

from nao_driver import NaoNode

from nao_msgs.msg import RunBehaviorAction

from nao_msgs.srv import (
    GetInstalledBehaviors, 
    GetInstalledBehaviorsResponse,
    )

class NaoBehaviors(NaoNode):
    #This should be treated as a constant
    NODE_NAME = "nao_behaviors"
    
    def __init__( self ):
        
        #Initialisation
        NaoNode.__init__( self, self.NODE_NAME )
        
        #We need this variable to be able to call stop behavior when preempted
        self.behavior = None
        self.lock = threading.RLock()
        
        #Proxy for listingBehaviors and stopping them
        self.behaviorProxy = self.get_proxy( "ALBehaviorManager" )
        
        # Register ROS services
        self.getInstalledBehaviorsService = rospy.Service(
            "get_installed_behaviors",
            GetInstalledBehaviors,
            self.getInstalledBehaviors
            )
        
        #Prepare and start actionlib server
        self.actionlibServer = actionlib.SimpleActionServer(
            "run_behavior",
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
        #Note this function is executed from a different thread
        rospy.logdebug(
            "Execution of behavior: '{}' requested".format(request.behavior))
        
        #Check requested behavior is installed
        if not request.behavior in self.behaviorProxy.getInstalledBehaviors():
            error_msg = "Behavior '{}' not installed".format(request.behavior)
            self.actionlibServer.set_aborted(text = error_msg)
            rospy.logdebug(error_msg)
            return
        
        with self.lock:
            # Check first if we're already preempted, and return if so
            if self.actionlibServer.is_preempt_requested():
                self.actionlibServer.set_preempted()
                rospy.logdebug("Behavior execution preempted before it started")
                return

            #Save name of behavior to be run
            self.behavior = request.behavior
            #Execute behavior (on another thread so we can release lock)
            taskID = self.behaviorProxy.post.runBehavior( self.behavior )

        # Wait for task to complete (or be preempted)    
        rospy.logdebug("Waiting for behavior execution to complete")
        self.behaviorProxy.wait( taskID, 0 )
        
        #Evaluate results
        with self.lock:
            self.behavior = None
            # If preempted, report so
            if self.actionlibServer.is_preempt_requested() :
                self.actionlibServer.set_preempted()
                rospy.logdebug("Behavior execution preempted")
            # Otherwise, set as succeeded
            else:
                self.actionlibServer.set_succeeded()
                rospy.logdebug("Behavior execution succeeded")

    def stopBehavior( self ):
        with self.lock:
            if self.behavior and self.actionlibServer.is_active() :
                self.behaviorProxy.stopBehavior( self.behavior )


if __name__ == '__main__':
    node = NaoBehaviors()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
