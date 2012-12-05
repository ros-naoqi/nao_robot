#!/usr/bin/env python

#
# ROS node to interface with Naoqi speech recognition and text-to-speech modules
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

from nao_driver import NaoNode
from naoqi import (ALBroker, ALProxy, ALModule)

from std_msgs.msg import( String )
from std_srvs.srv import( Empty, EmptyResponse )
from nao_msgs.msg import( WordRecognized )

class NaoSpeech(NaoNode):
    #This should be treated as a constant
    NODE_NAME = "nao_speech"   
    
    def __init__( self ):
        #Initialisation
        NaoNode.__init__( self )
        rospy.init_node( self.NODE_NAME )
        
        #Variable to keep track of subscription to Naoqi's speech recognition
        self.subscribed = False
        
        # Get text-to-speech proxy
        # Speech-recognition wrapper will be lazily initialized
        self.tts = self.getProxy("ALTextToSpeech")
        self.srw = None
        
        #Update parameters from ROS Parameter server
        self.reconfigure( None )
        
        #Subscribe to speech topic
        self.sub = rospy.Subscriber("speech", String, self.say )
        
        # Advertise word recognise topic
        self.pub = rospy.Publisher("word_recognized", WordRecognized )
        
        # Register ROS services
        self.reconfigure_srv = rospy.Service(
            self.NODE_NAME + "/reconfigure",
            Empty,
            self.reconfigure
            )
            
        self.start_srv = rospy.Service(
            self.NODE_NAME + "/start_recognition",
            Empty,
            self.start )
        
        self.stop_srv = rospy.Service(
            self.NODE_NAME + "/stop_recognition",
            Empty,
            self.stop )

    # DESTRUCTOR
    def __del__( self ):
        if self.subscribed:
            self.stop( None )
    
    # RETRIEVE PARAMETERS FROM PARAMETER SERVER AND UPDATE PROGRAM
    def reconfigure( self, request ):
        self.voice = (
            str( rospy.get_param("~voice") ) 
            if rospy.has_param("~voice") 
            else self.tts.getVoice() )

        self.language = (
            str( rospy.get_param("~language") )
            if rospy.has_param("~language")
            else self.tts.getLanguage() )
      
        self.volume = (
            float( rospy.get_param("~volume") )
            if rospy.has_param("~volume")
            else self.tts.getVolume() )
        
        self.vocabulary = (
            list(rospy.get_param("~vocabulary") )
            if rospy.has_param("~vocabulary")
            else [] )
            
        self.audio_expression = (
            bool(rospy.get_param("~enable_audio_expression"))
            if rospy.has_param("~enable_audio_expression")
            else None )
        
        self.visual_expression = ( 
            bool(rospy.get_param("~enable_visual_expression"))
            if rospy.has_param("~enable_visual_expression")
            else None )
        
        self.word_spotting = (
            bool(rospy.get_param("~enable_word_spotting"))
            if rospy.has_param("~enable_word_spotting")
            else False  )
        
        #if subscribed reconfigure speech recognition wrapper by restarting it
        if self.subscribed:
            stop( None )
            start( None )
        
        return EmptyResponse()
        
    # CALLBACK FOR SPEECH METHOD
    def say( self, request ):
        #Get current voice parameters
        current_voice = self.tts.getVoice()
        current_language = self.tts.getLanguage()
        current_volume = self.tts.getVolume()
        
        #Modify them if needed
        if self.voice != current_voice:
            self.tts.setVoice( self.voice )
        
        if self.language != current_language:
            self.tts.setLanguage( self.language )
        
        if self.volume != current_volume:
            self.tts.setVolume( self.volume )
        
        #Say whatever we need to say        
        self.tts.say( request.data )
        
        #Restore them
        if self.voice != current_voice:
            self.tts.setVoice( current_voice )
        
        if self.language != current_language:
            self.tts.setLanguage( current_language )
        
        if self.volume != current_volume:
            self.tts.setVolume( current_volume )
    
    # SPEECH RECOGNITION CALLBACK
    def start( self, request ):
        #Before starting make sure we have the wrapper ready
        if not self.srw:
            self.srw = SpeechRecognitionWrapper(self.pip, self.pport, self.pub)
        
        #Check we're not already subscribed
        if self.subscribed:
            rospy.logerr("Speech recognition already started")
            return EmptyResponse()
        
        #Check no one else has started module
        if self.srw.memory.getSubscribers("WordRecognized"):
            rospy.logerr("Speech recognition in use by another node")
            return EmptyResponse()
        
        if not self.vocabulary:
            rospy.logerr("No vocabulary specified for speech recognition")
            return EmptyResponse()
        
        if not self.audio_expression is None: 
            self.srw.proxy.setAudioExpression( self.audio_expression )
        
        if not self.visual_expression is None:
            self.srw.proxy.setVisualExpression( self.visual_expression )
        
        #Set language and vocabulary
        self.srw.proxy.setLanguage( self.language )
        self.srw.proxy.setVocabulary( self.vocabulary, self.word_spotting )
                
        #Set NaoQi callback to go to the speech recognition wrapper
        self.srw.memory.subscribeToEvent(
            "WordRecognized",
            self.srw.naoqi_name,
            "on_word_recognised"
            )
                    
        self.subscribed = True
        
        return EmptyResponse()
        
    def stop( self, request ):
        if not self.subscribed:
            rospy.logerr("Speech recognition wasn't started")
            return EmptyResponse()
        
        self.srw.memory.unsubscribeToEvent( 
            "WordRecognized",
            self.srw.naoqi_name
            )
        self.subscribed = False
        
        return EmptyResponse()
        

#This class is meant to be used only by NaoSpeech
#The speech recognition wrapper is lazily initialised       
class SpeechRecognitionWrapper(ALModule):
    """ROS wrapper for Naoqi speech recognition"""
    def __init__(self, ip, port, publisher):
        
        # Get a (unique) name for naoqi module which is based on the node name
        # and is a valid Python identifier (will be useful later)
        self.naoqi_name = "ros" + rospy.get_name().replace("/", "_")
        
        #Start ALBroker (needed by ALModule)
        self.broker = ALBroker(self.naoqi_name + "_broker",
            "0.0.0.0",   # listen to anyone
            0,           # find a free port and use it
            ip,    # parent broker IP
            port   # parent broker port
            )
        
        #Init superclassALModule
        ALModule.__init__( self, self.naoqi_name )
        
        self.memory = ALProxy("ALMemory")
        self.proxy = ALProxy("ALSpeechRecognition")
        
        #Keep publisher to send word recognized
        self.pub = publisher
        
        #Install global variables needed by Naoqi
        self.install_naoqi_globals()
        
    # Install global variables needed for Naoqi callbacks to work
    def install_naoqi_globals(self):
        globals()[self.naoqi_name] = self
        globals()["memory"] = self.memory
        
    def on_word_recognised(self, key, value, subscriber_id ):
        """Publish the words recognized by NAO via ROS """
        
        #Create dictionary, by grouping into tuples the list in value
        temp_dict = dict( value[i:i+2] for i in range(0, len(value), 2) )

        #Delete empty string from dictionary
        if '' in temp_dict: 
            del(temp_dict[''])
        
        self.pub.publish(WordRecognized( temp_dict.keys(), temp_dict.values() ))


if __name__ == '__main__':
    node = NaoSpeech()
    rospy.loginfo( node.NODE_NAME + " running..." )
    
    rospy.spin()
    
    #If speech recognition was started make sure we stop it
    if node.subscribed:
        node.stop( None )
    rospy.loginfo( node.NODE_NAME + " stopped." )
    
    exit(0)
