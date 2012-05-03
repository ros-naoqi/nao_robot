/*
# ROS node to read Nao's sensors and torso odometry through the Aldebaran API.
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2011 Daniel Maier, University of Freiburg
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
*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nao_msgs/TorsoOdometry.h>
#include <nao_msgs/TorsoIMU.h>
#include <iostream>
// Aldebaran includes
#include <almemoryproxy.h>
#include <almotionproxy.h>
#include <alvisiondefinitions.h>
#include <alerror.h>
#include <alvalue.h>
#include <altoolsmain.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/alproxy.h>
#include <alproxies/alproxies.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>



// Other includes
#include <boost/program_options.hpp>

using namespace std;
/*
from nao_driver import *

import threading
from threading import Thread

*/
class NaoNode
{
   public:
      NaoNode();
      ~NaoNode();
      bool connectNaoQi();
      void parse_command_line(int argc, char ** argv);
   protected:
      std::string m_pip;
      std::string m_ip;
      int m_port;
      int m_pport;
      std::string m_brokerName;
      boost::shared_ptr<AL::ALBroker> m_broker;


};


NaoNode::NaoNode() : m_pip("127.0.0.01"),m_ip("0.0.0.0"),m_port(16712),m_pport(9559),m_brokerName("NaoROSBroker")
{


}

NaoNode::~NaoNode()
{
}

void NaoNode::parse_command_line(int argc, char ** argv)
{
   std::string pip;
   std::string ip;
   int pport;
   int port;
   boost::program_options::options_description desc("Configuration");
   desc.add_options()
      ("help", "show this help message")
      ("ip", boost::program_options::value<std::string>(&ip)->default_value(m_ip),
       "IP/hostname of the broker")
      ("port", boost::program_options::value<int>(&port)->default_value(m_port),
       "Port of the broker")
      ("pip", boost::program_options::value<std::string>(&pip)->default_value(m_pip),
       "IP/hostname of parent broker")
      ("pport", boost::program_options::value<int>(&pport)->default_value(m_pport),
       "port of parent broker")
      ;
   boost::program_options::variables_map vm;
   boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
   boost::program_options::notify(vm);
   m_port = vm["port"].as<int>();
   m_pport = vm["pport"].as<int>();
   m_pip = vm["pip"].as<std::string>();
   m_ip = vm["ip"].as<std::string>();
   cout << "pip is " << m_pip << endl;
   cout << "ip is " << m_ip << endl;
   cout << "port is " << m_port << endl;
   cout << "pport is " << m_pport << endl;

   if (vm.count("help")) {
      std::cout << desc << "\n";
      return ;
   }
}



bool NaoNode::connectNaoQi()
{
   // Need this to for SOAP serialization of floats to work
   setlocale(LC_NUMERIC, "C");
   // A broker needs a name, an IP and a port:
   // FIXME: would be a good idea to look for a free port first
   // listen port of the broker (here an anything)
   try
   {
      m_broker = AL::ALBroker::createBroker(m_brokerName, m_ip, m_port, m_pip, m_pport, false);
   }
   catch(const AL::ALError& e) 
   {
      ROS_ERROR( "Failed to connect broker to: %s:%d",m_pip.c_str(),m_port);
      //AL::ALBrokerManager::getInstance()->killAllBroker();
      //AL::ALBrokerManager::kill();
      return false;
   }
   cout << "broker ready." << endl;
   return true;
}

class NaoSensors : public NaoNode
{
   public:
      //NaoSensors()  ;
      NaoSensors(int argc, char ** argv);
      ~NaoSensors();

      bool connectProxy();
      void run();
   protected:

      double m_rate;
      boost::shared_ptr<AL::ALMotionProxy> m_motionProxy;
      boost::shared_ptr<AL::ALMemoryProxy> m_memoryProxy;
      AL::ALValue m_dataNamesList;
      ros::NodeHandle m_nh;
      ros::NodeHandle m_privateNh;
      bool m_send_cam_odom;
      std::string m_tf_prefix;
      std::string m_odom_frame_id;
      std::string m_base_frame_id;
      nao_msgs::TorsoOdometry m_torsoOdom;
      nao_msgs::TorsoOdometry m_camOdom;
      nao_msgs::TorsoIMU m_torsoIMU;
      sensor_msgs::JointState m_jointState;
      ros::Publisher m_torsoOdomPub;
      ros::Publisher m_camOdomPub;
      ros::Publisher m_torsoIMUPub;
      ros::Publisher m_jointStatePub;
};

bool NaoSensors::connectProxy()
{
   if (!m_broker)
   {
      ROS_ERROR("Broker is not ready. Have you called connectNaoQi()?");
      return false;
   }
   try
   {
      //m_motionProxy = boost::shared_ptr<AL::ALProxy>(m_broker->getProxy("ALMotion"));
      m_motionProxy = boost::shared_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(m_broker));
   }
   catch (const AL::ALError& e) 
   {
      ROS_ERROR("Could not create ALMotionProxy.");
      return false;
   }
   try
   {
      //m_memoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(m_broker->getProxy("ALMemory"));
      m_memoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(m_broker));
   }
   catch (const AL::ALError& e)
   {
      ROS_ERROR("Could not create ALMemoryProxy.");
      return false;
   }
   ROS_INFO("Proxies to ALMotion and ALMemory ready.");
   return true;
}

NaoSensors::NaoSensors(int argc, char ** argv)
 : m_rate(50.0), m_privateNh("~"),
   m_send_cam_odom(false),
   m_tf_prefix(""),
   m_odom_frame_id("odom"),
   m_base_frame_id("torso")
{
   parse_command_line(argc,argv);
   if (   ! connectNaoQi() || !  connectProxy() )
   {
      ROS_ERROR("Gosh! Throwsing exception");
      throw std::exception();
   }
   m_dataNamesList = AL::ALValue::array("DCM/Time",
         "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value","Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
         "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value", "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value",
         "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value", "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value",
         "Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value");


   // get update frequency
   m_privateNh.param("torso_odom_rate", m_rate,m_rate);
   // get tf prefix
   std::string tf_prefix_param_name;
   if ( m_nh.searchParam("tf_prefix", tf_prefix_param_name) )
   {
      m_nh.param(tf_prefix_param_name,m_tf_prefix,m_tf_prefix);
   }
   else
   {
      ROS_WARN("Could not find tf_prefix, using \"%s\" instead",m_tf_prefix.c_str());
   }
   // get base_frame_id (and fix prefixi f necessary)
   m_privateNh.param("base_frame_id", m_base_frame_id, m_base_frame_id);
   if (m_base_frame_id[0] != '/')
      m_base_frame_id = m_tf_prefix + '/' + m_base_frame_id;

   // check if we need to send camera odometry
   m_privateNh.param("send_cam_odom", m_send_cam_odom, m_send_cam_odom);
   // initialize messages
   /*
      m_torsoOdom = nao_msgs::TorsoOdometry;
      m_camOdom = nao_msgs::TorsoOdometry;
      m_torsoIMU = nao_msgs::TorsoIMU;
      m_jointState = nao_msgs::JointState;
      */
   m_privateNh.param("odom_frame_id", m_odom_frame_id, m_odom_frame_id);
   if (m_odom_frame_id[0] != '/')
   {
      m_odom_frame_id = m_tf_prefix + '/' + m_odom_frame_id;
   }
   m_torsoOdom.header.frame_id = m_odom_frame_id;
   m_camOdom.header.frame_id = m_odom_frame_id;
   m_torsoIMU.header.frame_id = m_base_frame_id;
   m_jointState.name = m_motionProxy->getJointNames("Body");

   // simluated model misses some joints, we need to fill:
   if (m_jointState.name.size() == 22)
   {
      // TODO: Check this!
      m_jointState.name.insert(m_jointState.name.begin()+6,"LWristYaw");
      m_jointState.name.insert(m_jointState.name.begin()+7,"LHand");
      m_jointState.name.push_back("RWristYaw");
      m_jointState.name.push_back("RHand");
   }

   std::stringstream ss;
   ss << "Nao joints found: " ;
   std::copy(m_jointState.name.begin(), m_jointState.name.end()-1, std::ostream_iterator<std::string>(ss,",")); 
   std::copy(m_jointState.name.end()-1, m_jointState.name.end(), std::ostream_iterator<std::string>(ss)); 
   ROS_INFO("Nao joints found: %s",ss.str().c_str());


   if (m_send_cam_odom)
      m_camOdomPub = m_nh.advertise<nao_msgs::TorsoOdometry>("camera_odometry",5);
   m_torsoOdomPub = m_nh.advertise<nao_msgs::TorsoOdometry>("torso_odometry",5);
   m_torsoIMUPub = m_nh.advertise<nao_msgs::TorsoIMU>("torso_imu",5);
   m_jointStatePub = m_nh.advertise<sensor_msgs::JointState>("joint_states",5);

   ROS_INFO("nao_sensors initialized");

}
NaoSensors::~NaoSensors()
{
}
void NaoSensors::run()
{
   ros::Rate r(m_rate);
   ros::Time stamp1;
   ros::Time stamp2;
   ros::Time stamp;
   std::vector<float> odomData;
   std::vector<float> camData;
   std::vector<float> memData;
   std::vector<float> positionData;
   ROS_INFO("Staring main loop. ros::ok() is %d nh.ok() is %d",ros::ok(),m_nh.ok());
   while(ros::ok() )
   {
      r.sleep();
      ros::spinOnce();
      stamp1 = ros::Time::now();
      try
      {
         memData = m_memoryProxy->getListData(m_dataNamesList);
         // {SPACE_TORSO = 0, SPACE_WORLD = 1, SPACE_NAO = 2}. (second argument)
         odomData = m_motionProxy->getPosition("Torso", 1, true);
         if (m_send_cam_odom)
            camData = m_motionProxy->getPosition("CameraTop", 1, true);
         positionData = m_motionProxy->getAngles("Body", true);
      }
      catch (const AL::ALError & e)
      {
         ROS_ERROR( "Error accessing ALMemory, exiting...");
         ROS_ERROR( "%s", e.what() );
         //ros.signal_shutdown("No NaoQI available anymore");
      }
      stamp2 = ros::Time::now();
      //ROS_DEBUG("dt is %f",(stamp2-stamp1).toSec()); % dt is typically around 1/1000 sec
      // TODO: Something smarter than this..
      stamp = stamp1 + ros::Duration((stamp2-stamp1).toSec()/2.0);

      if (odomData.size()!=6)
      {
         ROS_ERROR( "Error getting odom data. length is %u",odomData.size() );
         continue;
      }
      if (m_send_cam_odom && camData.size()!=6)
      {
         ROS_ERROR( "Error getting camera odom data. length is %u",camData.size() );
         continue;
      }
      m_torsoOdom.header.stamp = stamp;
      m_torsoOdom.x = odomData[0];
      m_torsoOdom.y = odomData[1];
      m_torsoOdom.z = odomData[2];
      m_torsoOdom.wx = odomData[3];
      m_torsoOdom.wy = odomData[4];
      m_torsoOdom.wz = odomData[5];

      if (m_send_cam_odom)
      {
         m_camOdom.header.stamp = stamp;
         m_camOdom.x = camData[0];
         m_camOdom.y = camData[1];
         m_camOdom.z = camData[2];
         m_camOdom.wx = camData[3];
         m_camOdom.wy = camData[4];
         m_camOdom.wz = camData[5];
      }

      m_torsoOdomPub.publish(m_torsoOdom);
      if (m_send_cam_odom)
         m_camOdomPub.publish(m_camOdom);

      // Replace 'None' values with 0
      // (=> consistent behavior in 1.8 / 1.10 with 1.6)
      // Should not be necessary because NULL==0, but I'll leave it here..
      /*
      for (unsigned i = 0; i<memData.size(); ++i)
      {
         if (memData[i]==NULL)
            memData[i] = 0;
      }
      */
      if (memData.size() != m_dataNamesList.getSize())
      {
         ROS_ERROR("memData length %u does not match expected length %u",memData.size(),m_dataNamesList.getSize() );
         continue;
      }
      // IMU data:
      m_torsoIMU.header.stamp = stamp;
      m_torsoIMU.angleX = memData[1];
      m_torsoIMU.angleY = memData[2];
      m_torsoIMU.gyroX = memData[3];
      m_torsoIMU.gyroY = memData[4];
      m_torsoIMU.accelX = memData[5];
      m_torsoIMU.accelY = memData[6];
      m_torsoIMU.accelZ = memData[7];

      m_torsoIMUPub.publish(m_torsoIMU);

      // Joint States:
      m_jointState.header.stamp = stamp;
      m_jointState.header.frame_id = m_base_frame_id;
      m_jointState.position.resize(positionData.size());
      for(unsigned i = 0; i<positionData.size(); ++i)
      {
         m_jointState.position[i] = positionData[i];
      }

      // simulated model misses some joints, we need to fill:
      if (m_jointState.position.size() == 22)
      {
         m_jointState.position.insert(m_jointState.position.begin()+6,0.0);
         m_jointState.position.insert(m_jointState.position.begin()+7,0.0);
         m_jointState.position.push_back(0.0);
         m_jointState.position.push_back(0.0);
      }

      m_jointStatePub.publish(m_jointState);

   }
   ROS_INFO("nao_sensors stopped.");

}


int main(int argc, char ** argv)
{
   ros::init(argc, argv, "nao_sensors_cpp");
   cout << "I am here" << endl;
   NaoSensors * sensors;
   try{
      sensors = new NaoSensors(argc,argv);
      //NaoSensors sensors(argc,argv);
   }
   catch (const std::exception & e)
      //catch (...)
   {
      // TODO: why does this not work?
      //ROS_ERROR("Creating NaoSensors object failed with error %s",e.what());
      ROS_ERROR("Creating NaoSensors object failed with error ");
      return -1;
   }
   //sensors.parse_command_line(argc,argv);
   //sensors.connectNaoQi();
   //sensors.connectProxy();
   sensors->run();
   delete sensors;

   /*
      ROS_INFO("NodeHandle");
      ros::Rate r(50);

      while(ros::ok() )
      {
      r.sleep();
      ros::spinOnce();
      }
      ROS_INFO("nao_sensors stopped.");
      */
   ROS_INFO("End of the road.");
   return 0;
}
