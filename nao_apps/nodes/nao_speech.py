#!/usr/bin/env python

#
# ROS node to interface with Naoqi speech recognition and text-to-speech modules
# Tested with NaoQI: 1.12
#
# Copyright (c) 2012, 2013, Miguel Sarabia
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

import rospy
import actionlib

from dynamic_reconfigure.server import Server as ReConfServer
import dynamic_reconfigure.client
from naoqi_driver_py.cfg import NaoqiSpeechConfig as NodeConfig
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALBroker, ALProxy, ALModule)

from std_msgs.msg import( String )
from std_srvs.srv import( Empty, EmptyResponse )
from naoqi_bridge_msgs.msg import(
    WordRecognized,
    SetSpeechVocabularyGoal,
    SetSpeechVocabularyResult,
    SetSpeechVocabularyAction,
    SpeechWithFeedbackGoal,
    SpeechWithFeedbackResult,
    SpeechWithFeedbackFeedback,
    SpeechWithFeedbackAction )


class Constants:
    NODE_NAME = "nao_speech"
    EVENT = "WordRecognized"
    TEXT_STARTED_EVENT = "ALTextToSpeech/TextStarted"
    TEXT_DONE_EVENT = "ALTextToSpeech/TextDone"


class Util:
    @staticmethod
    def parse_vocabulary( vocabulary ):
        # Split string
        vocabulary_list = vocabulary.split("/")
        # Remove surrounding whitespace
        vocabulary_list = [ entry.strip() for entry in vocabulary_list]
        # Remove empty strings
        return filter(None, vocabulary_list)

    # Methods for name conversion
    @staticmethod
    def to_naoqi_name(name):
        return "ros{}_{}".format(
            name.replace("/", "_"),
            rospy.Time.now().to_sec() )

class DummyAudioDevice:
    def getOutputVolume(self):
        return 0

    def setOutputVolume(self, vol):
        pass

class NaoSpeech(ALModule, NaoqiNode):

    def __init__( self, moduleName ):
        # ROS Initialisation
        NaoqiNode.__init__(self, Constants.NODE_NAME )

        # NAOQi Module initialization
        self.moduleName = moduleName
        # Causes ALBroker to fill in ip and find a unused port
        self.ip = ""
        self.port = 0
        self.init_almodule()

        # Used for speech with feedback mode only
        self.speech_with_feedback_flag = False

        # State variables
        self.conf = None

        # Get Audio proxies
        # Speech-recognition wrapper will be lazily initialized
        self.srw = None

        # Subscription to the Proxy events
        self.subscribe()

        # Start reconfigure server
        self.reconf_server = ReConfServer(NodeConfig, self.reconfigure)
        # Client for receiving the new information
        self.reconf_client = dynamic_reconfigure.client.Client(rospy.get_name())

        #Subscribe to speech topic
        self.sub = rospy.Subscriber("speech", String, self.say )

        # Advertise word recognise topic
        self.pub = rospy.Publisher("word_recognized", WordRecognized )

        # Register ROS services
        self.start_srv = rospy.Service(
            "start_recognition",
            Empty,
            self.start )

        self.stop_srv = rospy.Service(
            "stop_recognition",
            Empty,
            self.stop )

        # Actionlib server for altering the speech recognition vocabulary
        self.setSpeechVocabularyServer = actionlib.SimpleActionServer("speech_vocabulary_action", SetSpeechVocabularyAction,
                                                                  execute_cb=self.executeSpeechVocabularyAction,
                                                                  auto_start=False)

        # Actionlib server for having speech with feedback
        self.speechWithFeedbackServer = actionlib.SimpleActionServer("speech_action", SpeechWithFeedbackAction,
                                                                  execute_cb=self.executeSpeechWithFeedbackAction,
                                                                  auto_start=False)
        # Start both actionlib servers
        self.setSpeechVocabularyServer.start()
        self.speechWithFeedbackServer.start()

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
            rospy.logerr("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)

        self.tts = self.get_proxy("ALTextToSpeech")
        # TODO: check self.memProxy.version() for > 1.6
        if self.tts is None:
            rospy.logerr("Could not get a proxy to ALTextToSpeech on %s:%d", self.pip, self.pport)
            exit(1)

        self.audio = self.get_proxy("ALAudioDevice")
        if self.audio is None:
            # When using simulated naoqi, audio device is not available,
            # Use a dummy instead
            rospy.logwarn("Proxy to ALAudioDevice not available, using dummy device (normal in simulation; volume controls disabled)")
            self.audio = DummyAudioDevice()

    def subscribe(self):
        # Subscription to the ALProxies events
        self.memProxy.subscribeToEvent(Constants.TEXT_DONE_EVENT, self.moduleName, "onTextDone")
        self.memProxy.subscribeToEvent(Constants.TEXT_STARTED_EVENT, self.moduleName, "onTextStarted")

    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent(Constants.TEXT_DONE_EVENT, self.moduleName)
        self.memProxy.unsubscribeToEvent(Constants.TEXT_STARTED_EVENT, self.moduleName)

    def onTextStarted(self, strVarName, value, strMessage):
        # Called when NAO begins or ends the speech. On begin the value = 1
        # Must work only on speech with feedback mode
        if value == 0 or self.speech_with_feedback_flag == False:
            return

        # Send feedback via the speech actionlib server
        fb = SpeechWithFeedbackFeedback()
        self.speechWithFeedbackServer.publish_feedback(fb)

    def onTextDone(self, strVarName, value, strMessage):
        # Called when NAO begins or ends the speech. On end the value = 1
        # Must work only on speech with feedback mode
        if value == 0 or self.speech_with_feedback_flag == False:
            return

        # Change the flag to inform the executeSpeechWithFeedbackAction function that
        # the speaking process is over
        self.speech_with_feedback_flag = False


    def executeSpeechWithFeedbackAction(self, goal):
        # Gets the goal and begins the speech
        self.speech_with_feedback_flag = True
        saystr = goal.say
        self.internalSay(saystr)

        # Wait till the onTextDone event is called or 2 mins are passed
        counter = 0
        while self.speech_with_feedback_flag == True and counter < 1200:
            rospy.sleep(0.1)
            counter += 1

        # Send the success feedback
        self.speechWithFeedbackServer.set_succeeded()

    def executeSpeechVocabularyAction(self, goal):
        #~ Called by action client
        rospy.loginfo("SetSpeechVocabulary action executing");

        words = goal.words
        words_str = ""

        #~ Empty word list. Send failure.
        if len(words) == 0:
            setVocabularyResult = SetSpeechVocabularyResult()
            setVocabularyResult.success = False
            self.setSpeechVocabularyServer.set_succeeded(setVocabularyResult)
            return

        #~ Create the vocabulary string
        for i in range(0, len(words) - 1):
            words_str += str(words[i]) + "/"

        words_str += words[len(words) - 1]

        #~ Update the dynamic reconfigure vocabulary parameter
        params = { 'vocabulary' : words_str }
        self.reconf_client.update_configuration(params)

        #~ Send success
        setVocabularyResult = SetSpeechVocabularyResult()
        setVocabularyResult.success = True
        self.setSpeechVocabularyServer.set_succeeded(setVocabularyResult)

    # RECONFIGURE THIS PROGRAM
    def reconfigure( self, request, level ):
        newConf = {}

        #Copy values
        newConf["voice"] = request["voice"]
        newConf["language"] = request["language"]
        newConf["volume"] = request["volume"]
        newConf["vocabulary"] = request["vocabulary"]
        newConf["audio_expression"] = request["audio_expression"]
        newConf["visual_expression"] = request["visual_expression"]
        newConf["word_spotting"] = request["word_spotting"]

        # Check and update values
        if not newConf["voice"]:
            newConf["voice"] = self.tts.getVoice()
        elif newConf["voice"] not in self.tts.getAvailableVoices():
            rospy.logwarn(
                "Unknown voice '{}'. Using current voice instead".format(
                    newConf["voice"] ) )
            rospy.loginfo("Voices available: {}".format(
                self.tts.getAvailableVoices()))
            newConf["voice"] = self.tts.getVoice()

        if not newConf["language"]:
            newConf["language"] = self.tts.getLanguage()
        elif newConf["language"] not in self.tts.getAvailableLanguages():
            newConf["language"] = self.tts.getLanguage()
            rospy.logwarn(
                "Unknown language '{}'. Using current language instead".format(
                    newConf["language"] ) )
            rospy.loginfo("Languages available: {}".format(
                self.tts.getAvailableLanguages()))

        # If first time and parameter not explicitly set
        if not self.conf and not rospy.has_param("~volume"):
            newConf["volume"] = self.audio.getOutputVolume()

        # if srw is running and the vocabulary request is invalid, ignore it
        if self.srw and not Util.parse_vocabulary(newConf["vocabulary"]):
            rospy.logwarn("Empty vocabulary. Using current vocabulary instead")
            newConf["vocabulary"] = self.conf["vocabulary"]

        # Check if we need to restart srw
        if self.srw and self.conf and (
            newConf["language"] != self.conf["language"] or
            newConf["vocabulary"] != self.conf["language"] or
            newConf["audio_expression"] != self.conf["audio_expression"] or
            newConf["visual_expression"] != self.conf["visual_expression"] or
            newConf["word_spotting"] != self.conf["word_spotting"] ):
            need_to_restart_speech = True
        else:
            need_to_restart_speech = False

        self.conf = newConf

        #If we have enabled the speech recognition wrapper, reconfigure it
        if need_to_restart_speech:
            self.stop()
            self.start()

        return self.conf


    # CALLBACK FOR SPEECH METHOD
    def say( self, request ):
        self.internalSay(request.data)

    # Used for internal use. Called to say one sentence either from the speech
    # action goal callback or message callback
    def internalSay( self, sentence ):
        #Get current voice parameters
        current_voice = self.tts.getVoice()
        current_language = self.tts.getLanguage()
        current_volume = self.audio.getOutputVolume()
        current_gain = self.tts.getVolume()
        target_gain = 1.0

        #Modify them if needed
        if self.conf["voice"] != current_voice:
            self.tts.setVoice( self.conf["voice"] )

        if self.conf["language"] != current_language:
            self.tts.setLanguage( self.conf["language"] )

        if self.conf["volume"] != current_volume:
            self.audio.setOutputVolume( self.conf["volume"] )

        if target_gain != current_gain:
            self.tts.setVolume(target_gain)

        #Say whatever it is Nao needs to say
        self.tts.say( sentence )

        #And restore them
        if self.conf["voice"] != current_voice:
            self.tts.setVoice( current_voice )

        if self.conf["language"] != current_language:
            self.tts.setLanguage( current_language )

        if self.conf["volume"] != current_volume:
            self.audio.setOutputVolume( current_volume )

        if target_gain != current_gain:
            self.tts.setVolume(current_gain)

    # SPEECH RECOGNITION SERVICES
    def start( self, request = None ):
        if self.srw:
            rospy.logwarn("Speech recognition already started. Restarting.")
            self.srw.close()
        # Start only if vocabulary is valid
        if Util.parse_vocabulary( self.conf["vocabulary"] ):
            self.srw = SpeechRecognitionWrapper(
                self.pip,
                self.pport,
                self.pub,
                self.conf )
        else:
            rospy.logwarn("Empty vocabulary. Ignoring request.")

        return EmptyResponse()

    def stop( self, request = None ):
        if not self.srw:
            rospy.logerr("Speech recognition was not started")
        else:
            self.srw.stop()
            self.srw = None

        return EmptyResponse()

    def shutdown(self):
        self.unsubscribe()
        # Shutting down broker seems to be not necessary any more
        # try:
        #     self.broker.shutdown()
        # except RuntimeError,e:
        #     rospy.logwarn("Could not shut down Python Broker: %s", e)


#This class is meant to be used only by NaoSpeech
#The speech recognition wrapper is lazily initialised
class SpeechRecognitionWrapper(ALModule):

    """ROS wrapper for Naoqi speech recognition"""
    def __init__(self, ip, port, publisher, config):

        # Get a (unique) name for naoqi module which is based on the node name
        # and is a valid Python identifier (will be useful later)
        self.naoqi_name = Util.to_naoqi_name( rospy.get_name() )

        #Start ALBroker (needed by ALModule)
        self.broker = ALBroker(self.naoqi_name + "_broker",
            "0.0.0.0",   # listen to anyone
            0,           # find a free port and use it
            ip,          # parent broker IP
            port )       # parent broker port


        #Init superclass ALModule
        ALModule.__init__( self, self.naoqi_name )

        # Start naoqi proxies
        self.memory = ALProxy("ALMemory")
        self.proxy = ALProxy("ALSpeechRecognition")

        #Keep publisher to send word recognized
        self.pub = publisher

        #Install global variables needed by Naoqi
        self.install_naoqi_globals()

        #Check no one else is subscribed to this event
        subscribers = self.memory.getSubscribers(Constants.EVENT)
        if subscribers:
            rospy.logwarn("Speech recognition already in use by another node")
            for module in subscribers:
                self.stop(module)

        # Configure this instance
        self.reconfigure(config)

        #And subscribe to the event raised by speech recognition
        rospy.loginfo("Subscribing '{}' to NAO speech recognition".format(
            self.naoqi_name) )
        self.memory.subscribeToEvent(
            Constants.EVENT,
            self.naoqi_name,
            self.on_word_recognised.func_name )


    # Install global variables needed for Naoqi callbacks to work
    def install_naoqi_globals(self):
        globals()[self.naoqi_name] = self
        globals()["memory"] = self.memory


    def reconfigure(self, config):
        self.proxy.setLanguage( config["language"] )
        self.proxy.setAudioExpression( config["audio_expression"] )
        self.proxy.setVisualExpression( config["visual_expression"] )
        self.proxy.setVocabulary(
            Util.parse_vocabulary( config["vocabulary"].encode('utf-8') ),
            config["word_spotting"] )


    def stop(self, module = None):
        if module is None:
            module = self.naoqi_name

        rospy.loginfo("Unsubscribing '{}' from NAO speech recognition".format(
            module))
        try:
            self.memory.unsubscribeToEvent( Constants.EVENT, module )
        except RuntimeError:
            rospy.logwarn("Could not unsubscribe from NAO speech recognition")


    def on_word_recognised(self, key, value, subscriber_id ):
        """Publish the words recognized by NAO via ROS """

        #Create dictionary, by grouping into tuples the list in value
        temp_dict = dict( value[i:i+2] for i in range(0, len(value), 2) )

        #Delete empty string from dictionary
        if '' in temp_dict:
            del(temp_dict[''])

        self.pub.publish(WordRecognized( temp_dict.keys(), temp_dict.values() ))


if __name__ == '__main__':

    ROSNaoSpeechModule = NaoSpeech("ROSNaoSpeechModule")
    rospy.loginfo( "ROSNaoSpeechModule running..." )

    rospy.spin()

    rospy.loginfo("Stopping ROSNaoSpeechModule ...")
    #If speech recognition was started make sure we stop it
    if ROSNaoSpeechModule.srw:
        ROSNaoSpeechModule.srw.stop()
    # Shutdown the module
    ROSNaoSpeechModule.shutdown();
    rospy.loginfo("ROSNaoSpeechModule stopped.")

    exit(0)
