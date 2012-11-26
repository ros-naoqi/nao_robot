#!/usr/bin/env python

#
# ROS node to control NAO's LEDs using NaoQI
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
import random
import copy

from nao_driver import NaoNode

from nao_msgs.msg import(
    BlinkAction,
    BlinkResult,
    BlinkFeedback,
    FadeRGB
    )

class NaoLeds(NaoNode):
    #This should be treated as a constant
    NODE_NAME = "nao_leds"
    
    def __init__( self ):
        
        #Initialisation
        NaoNode.__init__( self )
        rospy.init_node( self.NODE_NAME )
        
        #Proxy to interface with LEDs
        self.proxy = self.getProxy( "ALLeds" )
        
        #Seed python's random number generator
        random.seed( rospy.Time.now().to_nsec() )
        
        #Create a subscriber for the fade_rgb topic
        self.subscriber = rospy.Subscriber( 
            self.NODE_NAME+ "/fade_rgb",
            FadeRGB,
            self.fade_rgb)

        #Prepare and start actionlib server
        self.actionlib = actionlib.SimpleActionServer(
            self.NODE_NAME + "/blink",
            BlinkAction,
            self.run_blink,
            False
            )
        self.actionlib.start()
        
    
    def fade_rgb(self, request) :
        hexColor =  int(
            int(request.color.r) << 16 | 
            int(request.color.g) << 8 |
            int(request.color.b)
            )

        self.proxy.fadeRGB(
            request.led_name,
            hexColor,
            request.fade_duration.to_sec()
            )
    
    def run_blink( self, request ):
        #Note that this function is executed on a different thread

        third_of_duration = request.blink_duration / 3.0
        
        #Prepare background message
        bg_msg = FadeRGB();
        bg_msg.led_name = "FaceLeds"
        bg_msg.color = request.bg_color
        bg_msg.fade_duration = third_of_duration
        
        #Prepare a copy for blink_msg
        blink_msg = copy.deepcopy( bg_msg )
        
        #Construct result and feedback
        feedback = BlinkFeedback()
        result = BlinkResult()
        result.still_blinking = False

        #Check valid parameters
        bad_request = False
        reason = ""
        if not request.colors:
            bad_request = True
            reason = "No colors to blink were specified"
        elif request.blink_duration.to_sec() <= 0.0:
            bad_request = True
            reason = "Blink duration cannot be 0"
        elif request.blink_rate_mean <= 0.0 or request.blink_rate_sd <= 0.0:
            bad_request = True
            reason = "Invalid parameter for blink rate"
        
        if bad_request:
            rospy.logwarn("Bad Blink request: " + reason)
            self.actionlib.set_aborted(result, reason)
            return
        
        #Sleep time is drawn from a gaussian dist with these parameters
        sleep_mean = request.blink_rate_mean
        sleep_sd = request.blink_rate_sd
        max_sleep_time = sleep_mean + 3* sleep_sd #This is highly unlikely
        
        #Main blinking loop
        while (
            self.actionlib.is_active() and 
            not self.actionlib.is_new_goal_available() and
            not rospy.is_shutdown()
            ) :
            
            #Chose a blinking color at random
            blink_msg.color = random.choice( request.colors )

            #Send command (takes 1/3 duration to fade)
            self.fade_rgb( blink_msg )
            #Sleep (takes 1/3 duration)
            rospy.sleep( third_of_duration )
            #Fade to background (takes another 1/3 duration)
            self.fade_rgb( bg_msg )
            
            #Publish feedback
            feedback.last_color = blink_msg.color
            self.actionlib.publish_feedback( feedback )
            
            #Sleep for a random amount of time
            sleep_time = random.gauss( sleep_mean, sleep_sd )
            
            if sleep_time < 0:
                sleep_time = 0
            elif sleep_time > max_sleep_time : 
                sleep_time = max_sleep_time
                
            rospy.sleep( sleep_time )
        
        #If we were pre-empted by other request, make sure result shows this
        if self.actionlib.is_new_goal_available() :
            result.still_blinking = True
                
        # Task never completes so if we reach here, blink has been pre-empted
        self.actionlib.set_preempted( result )

if __name__ == '__main__':
    node = NaoLeds()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
