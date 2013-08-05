#!/usr/bin/env python

# This script collects diagnostic information on a Nao robot and publishes
# the information as DiagnosticArray messages.
#
# This script was written and tested with NaoQI 1.10.52 and will probably
# fail on 1.12+ as some ALMemory keys and device paths have changed.
#
# You can run this script either on the robot or on a remote machine,
# however, CPU temperature and network status will only be available if the
# script runs directly on the robot.
#

# Copyright 2011-2012 Stefan Osswald, University of Freiburg
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


import roslib
roslib.load_manifest('nao_diagnostic')
import rospy

import dbus
from dbus.exceptions import DBusException

from nao_driver import *
import threading
from threading import Thread

from diagnostic_msgs.msg import *

import naoqi

class NaoDiagnosticUpdater(NaoNode,Thread):
    def __init__(self):
        NaoNode.__init__(self)
        Thread.__init__(self)
        
        # ROS initialization:
        rospy.init_node('nao_diagnostic_updater')        
        self.connectNaoQi()
        self.stopThread = False

        # Read parameters:
        self.sleep = 1.0/rospy.get_param('~update_rate', 0.5)  # update rate in Hz
        self.warningThreshold = rospy.get_param('~warning_threshold', 68) # warning threshold for joint temperatures
        self.errorThreshold = rospy.get_param('~error_threshold', 74)     # error threshold for joint temperatures
        self.runsOnRobot = naoqi.ON_GEODE   # if temperature device files cannot be opened, this flag will be set to False later.

        # Lists with interesting ALMemory keys
        self.jointNamesList = self.motionProxy.getJointNames('Body')    
        self.jointTempPathsList = []
        self.jointStiffPathsList = []    
        for i in range(0, len(self.jointNamesList)):
            self.jointTempPathsList.append("Device/SubDeviceList/%s/Temperature/Sensor/Value" % self.jointNamesList[i])
            self.jointStiffPathsList.append("Device/SubDeviceList/%s/Hardness/Actuator/Value" % self.jointNamesList[i])

        self.batteryNamesList = ["Percentage", "Status"]
        self.batteryPathsList = ["Device/SubDeviceList/Battery/Charge/Sensor/Value", 
                                 "Device/SubDeviceList/Battery/Charge/Sensor/Status",
                                 "Device/SubDeviceList/Battery/Current/Sensor/Value"]
        
        self.deviceInfoList = ["Device/DeviceList/ChestBoard/BodyId"]
        deviceInfoData = self.memProxy.getListData(self.deviceInfoList)
        if(len(deviceInfoData) > 1 and isinstance(deviceInfoData[1], list)):
            deviceInfoData = deviceInfoData[1]
        if(deviceInfoData[0] is None):
            # No device info -> running in a simulation
            self.isSimulator = True
            if(self.pip.startswith("127.") or self.pip == "localhost"):
                # Replace localhost with hostname of the machine
                f = open("/etc/hostname")
                self.hardwareID = "%s:%d"%(f.readline().rstrip(), self.pport)
                f.close()
            else:
                self.hardwareID = "%s:%d"%(self.pip, self.pport)            
        else:
            self.hardwareID = deviceInfoData[0].rstrip()
            self.isSimulator = False
            
        # init. messages:        
        self.diagnosticPub = rospy.Publisher("diagnostics", DiagnosticArray)

        rospy.loginfo("nao_diagnostic_updater initialized")

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        """ Connects to NaoQi and gets proxies to ALMotion and ALMemory. """
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.motionProxy = self.getProxy("ALMotion")
        self.memProxy = self.getProxy("ALMemory")
        
    def run(self):
        """ Diagnostic thread code - collects and sends out diagnostic data. """
        while(not self.stopThread):
            timestamp = rospy.Time.now()
            try:
                # Get data from robot
                jointsTempData = self.memProxy.getListData(self.jointTempPathsList)
                jointsStiffData = self.memProxy.getListData(self.jointStiffPathsList)
                batteryData = self.memProxy.getListData(self.batteryPathsList)
                if isinstance(jointsTempData[1], list):
                    # Some naoqi versions provide data in [0, [data1, data2, ...], timestamp] format,
                    # others just as [data1, data2, ...]
                    # --> get data list                    
                    jointsTempData = jointsTempData[1]
                    jointsStiffData = jointsStiffData[1]
                    batteryData = batteryData[1]
            except RuntimeError,e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

            diagnosticArray = DiagnosticArray()
            
            # Process joint temperature and stiffness
            for i in range(0, len(self.jointTempPathsList)):
                status = DiagnosticStatus()
                status.hardware_id = "%s#%s"%(self.hardwareID, self.jointNamesList[i])           
                status.name = "nao_joint: %s" % self.jointNamesList[i]                
                kv = KeyValue()
                kv.key = "Temperature"
                temperature = jointsTempData[i]
                kv.value = str(temperature)
                status.values.append(kv)
                kv = KeyValue()
                kv.key = "Stiffness"
                if self.jointNamesList[i] == "RHipYawPitch":
                    # same joint as LHipYawPitch, does not provide data
                    kv.value = "None"
                else:
                    kv.value = str(jointsStiffData[i])
                status.values.append(kv)
                if (type(temperature) != float and type(temperature) != int) or self.jointNamesList[i] == "RHipYawPitch":
                    status.level = DiagnosticStatus.OK
                    status.message = "No temperature sensor"
                elif temperature < self.warningThreshold:
                    status.level = DiagnosticStatus.OK
                    status.message = "Normal: %3.1f C" % temperature
                elif temperature < self.errorThreshold:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Hot: %3.1f C" % temperature
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Too hot: %3.1f C" % temperature
                diagnosticArray.status.append(status)
                        
            # Process battery status flags
            status = DiagnosticStatus()
            status.hardware_id = "%s#%s"%(self.hardwareID, "battery")
            status.name ="nao_power: Battery"
            kv = KeyValue()
            kv.key = "Percentage"
            if batteryData[0] is None:
                kv.value = "unknown"
            else:
                kv.value = str(batteryData[0] * 100)
            status.values.append(kv)

            # Battery status: See http://www.aldebaran-robotics.com/documentation/naoqi/sensors/dcm/pref_file_architecture.html?highlight=discharge#charge-for-the-battery
            # Note: SANYO batteries use different keys!
            statuskeys = ["End off Discharge flag", "Near End Off Discharge flag", "Charge FET on", "Discharge FET on", "Accu learn flag", "Discharging flag", "Full Charge Flag", "Charge Flag", "Charge Temperature Alarm", "Over Charge Alarm", "Discharge Alarm", "Charge Over Current Alarm", "Discharge Over Current Alarm (14A)", "Discharge Over Current Alarm (6A)", "Discharge Temperature Alarm", "Power-Supply present" ]
                        
            for j in range(0, 16):
                kv = KeyValue()
                kv.key = statuskeys[j]
                if batteryData[1] is None:
                    kv.value = "unknown"
                elif int(batteryData[1]) & 1<<j:
                    kv.value = "True"
                else:
                    kv.value = "False"
                status.values.append(kv)
                
            kv = KeyValue()
            kv.key = "Status"
            if batteryData[1] is None:
                kv.value = "unknown"
            else:
                kv.value = bin(batteryData[1])
            status.values.append(kv)

            # Process battery charge level
            if batteryData[0] is None:
                status.level = DiagnosticStatus.OK
                status.message = "Battery status unknown, assuming simulation"
            elif int(batteryData[1]) & 1<<6:
                status.level = DiagnosticStatus.OK
                status.message = "Battery fully charged"
            elif int(batteryData[1]) & 1<<7:
                status.level = DiagnosticStatus.OK
                status.message = "Charging (%3.1f%%)" % (batteryData[0] * 100)
            elif batteryData[0] > 0.60:
                status.level = DiagnosticStatus.OK
                status.message = "Battery OK (%3.1f%% left)" % (batteryData[0] * 100)
            elif batteryData[0] > 0.30:
                status.level = DiagnosticStatus.WARN
                status.message = "Battery discharging (%3.1f%% left)" % (batteryData[0] * 100)
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = "Battery almost empty (%3.1f%% left)" % (batteryData[0] * 100)
            diagnosticArray.status.append(status)
            

            # Process battery current
            status = DiagnosticStatus()
            status.hardware_id = "%s#%s"%(self.hardwareID, "power")
            status.name = "nao_power: Current"
             
            if batteryData[2] is None:
                status.level = DiagnosticStatus.OK
                status.message = "Total Current: unknown"
            else:
                kv = KeyValue()
                kv.key = "Current"
                kv.value = str(batteryData[2])
                status.values.append(kv)
                status.level = DiagnosticStatus.OK
                if batteryData[2] > 0:
                    currentStatus = "charging"
                else:
                    currentStatus = "discharging"
                status.message = "Total Current: %3.2f Ampere (%s)" % (batteryData[2], currentStatus)
            diagnosticArray.status.append(status)

            # Process CPU and head temperature
            status = DiagnosticStatus()
            status.hardware_id = "%s#%s"%(self.hardwareID, "cpu")
            status.name = "nao_cpu: Head Temperature"
            temp = self.temp_get()
            kv = KeyValue()
            kv.key = "CPU Temperature"
            kv.value = str(temp['HeadSilicium'])
            status.values.append(kv)
            kv = KeyValue()
            kv.key = "Board Temperature"
            kv.value = str(temp['HeadBoard'])            
            status.values.append(kv)
            if(temp['HeadSilicium'] == "invalid"):
                status.level = DiagnosticStatus.OK
                status.message = "unknown, assuming simulation"
            else:
                status.message = "%3.2f deg C" % (temp['HeadSilicium'])
                if temp['HeadSilicium'] < 100:
                    status.level = DiagnosticStatus.OK
                elif temp['HeadSilicium'] < 105:
                    status.level = DiagnosticStatus.WARN
                else:
                    status.level = DiagnosticStatus.ERROR                    
            diagnosticArray.status.append(status)            
            
            
            # Process WIFI and LAN status
            statusWifi = DiagnosticStatus()
            statusWifi.hardware_id = "%s#%s"%(self.hardwareID, "wlan")
            statusWifi.name = "nao_wlan: Status"
            
            statusLan = DiagnosticStatus()
            statusLan.hardware_id = "%s#%s"%(self.hardwareID, "ethernet")
            statusLan.name = "nao_ethernet: Status"
            
            if self.runsOnRobot:
                statusLan.level = DiagnosticStatus.ERROR
                statusLan.message = "offline"
                statusWifi.level = DiagnosticStatus.ERROR
                statusWifi.message = "offline"                              
                systemBus = dbus.SystemBus()
                try:
                    manager = dbus.Interface(systemBus.get_object("org.moblin.connman", "/"), "org.moblin.connman.Manager")
                except DBusException as e:
                    self.runsOnRobot = False
                if self.runsOnRobot:
                    properties = manager.GetProperties()
                    for property in properties["Services"]:
                        service = dbus.Interface(systemBus.get_object("org.moblin.connman", property), "org.moblin.connman.Service")
                        try:
                            props = service.GetProperties()
                        except DBusException as e:
                            continue # WLAN network probably disappeared meanwhile
                        
                        state = str(props.get("State", "None"))
                        if state == "idle":
                            pass # other network, not connected 
                        else:                 
                            nettype = str(props.get("Type", "<unknown>"))
                            if(nettype == "wifi"):
                                status = statusWifi
                            else:
                                status = statusLan   
                            kv = KeyValue()
                            kv.key = "Network"
                            kv.value = str(props.get("Name", "<unknown>"))                    
                            status.values.append(kv)
                            if nettype == "wifi":
                                strength = int(props.get("Strength", "<unknown>"))
                                kv = KeyValue()
                                kv.key = "Strength"
                                kv.value = "%d%%"%strength
                                status.values.append(kv)
                            else:
                                strength = None                    
                            kv = KeyValue()
                            kv.key = "Type"
                            kv.value = nettype
                            status.values.append(kv)
                            if strength is None:
                                status.message = state
                            else:
                                status.message = "%s (%d%%)"%(state, strength)
                                
                            if state in ["online", "ready"]:
                                status.level = DiagnosticStatus.OK
                            elif state in ["configuration", "association"]:
                                status.level = DiagnosticStatus.WARN
                            else: # can only be 'failure'
                                status.message = str("%s (%s)"%(state, props.get("Error")))
                                status.level = DiagnosticStatus.ERROR
            else:
                statusWifi.level = DiagnosticStatus.OK
                statusWifi.message = "nao_diagnostic_updater not running on robot, cannot determine WLAN status"
                                
                statusLan.level = DiagnosticStatus.OK
                statusLan.message = "nao_diagnostic_updater not running on robot, cannot determine Ethernet status"
                                        
            diagnosticArray.status.append(statusWifi)   
            diagnosticArray.status.append(statusLan)   
            
            # Publish all diagnostic messages
            diagnosticArray.header.stamp = rospy.Time.now()                                  
            self.diagnosticPub.publish(diagnosticArray)                             
            rospy.sleep(self.sleep)
            
    def temp_get(self):
        """Read the CPU and head temperature from the system devices.

        Returns {'HeadSilicium': <temperature>, 'HeadBoard': <temperature>}

        Temperatures are returned as float values in degrees Celsius, or
        as the string 'invalid' if the sensors are not accessible.
        """
        temp = {'HeadSilicium': 'invalid', 'HeadBoard': 'invalid'}
        if(self.runsOnRobot):
            try:
                f = open("/sys/class/i2c-adapter/i2c-1/1-004c/temp2_input")
                temp['HeadSilicium'] = float(f.readline()) / 1000.
                f.close()
            except IOError:
                self.runsOnRobot = False
                return temp
            except:
                temp['HeadSilicium'] = "invalid"
            try:
                f = open("/sys/class/i2c-adapter/i2c-1/1-004c/temp1_input")
                temp['HeadBoard'] = float(f.readline()) / 1000.
                f.close()
            except IOError:
                self.runsOnRobot = False
                return temp
            except:
                temp['HeadBoard'] = "invalid"
        return temp


if __name__ == '__main__':
    
    updater = NaoDiagnosticUpdater()
    updater.start()
    
    rospy.spin()
    
    rospy.loginfo("Stopping nao_diagnostic_updater ...")
    updater.stopThread = True
    updater.join()
    
    rospy.loginfo("nao_diagnostic_updater stopped.")
    exit(0)
