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
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

#include <std_srvs/Empty.h>
#include <nao_msgs/SetTransform.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// Aldebaran includes
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alvalue/alvalue.h>
#include <alcommon/altoolsmain.h>
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


NaoNode::NaoNode() : m_pip("127.0.0.01"),m_ip("0.0.0.0"),m_port(0),m_pport(9559),m_brokerName("NaoROSBroker")
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

    NaoSensors(int argc, char ** argv);
    ~NaoSensors();

    bool connectProxy();
    void run();

    bool pauseOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool resumeOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool odomOffsetCallback(nao_msgs::SetTransform::Request& req, nao_msgs::SetTransform::Response& res);
    bool setOdomPoseCallback(nao_msgs::SetTransform::Request& req, nao_msgs::SetTransform::Response& res);


protected:

    double m_rate;

    // NAOqi
    boost::shared_ptr<AL::ALMotionProxy> m_motionProxy;
    boost::shared_ptr<AL::ALMemoryProxy> m_memoryProxy;
    AL::ALValue m_dataNamesList;

    // ROS
    ros::NodeHandle m_nh;
    ros::NodeHandle m_privateNh;

    // Services
    ros::ServiceServer m_pauseOdomSrv;
    ros::ServiceServer m_resumeOdomSrv;
    ros::ServiceServer m_odomOffsetSrv;
    ros::ServiceServer m_setOdomPoseSrv;

    std::string m_odomFrameId;
    std::string m_baseFrameId;

    nav_msgs::Odometry m_odom;
    sensor_msgs::Imu m_torsoIMU;
    sensor_msgs::JointState m_jointState;

    ros::Publisher m_odomPub;
    tf::TransformBroadcaster m_transformBroadcaster;
    ros::Publisher m_torsoIMUPub;
    ros::Publisher m_jointStatePub;

    // Odometry-specific members
    geometry_msgs::TransformStamped m_odomTransformMsg;
    tf::Pose m_odomPose; // current "real" odometry pose in original (Nao) odom frame
    tf::Transform m_odomOffset; // offset on odometry origin

    bool m_useIMUAngles;

    bool m_paused;
    double m_lastOdomTime;

    tf::Pose m_targetPose;
    bool m_mustUpdateOffset;
    bool m_initializeFromIMU;
    bool m_initializeFromOdometry;
    bool m_isInitialized;
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
 : m_rate(25.0), m_privateNh("~"),
   m_odomFrameId("odom"),
   m_baseFrameId("base_link"),
   m_useIMUAngles(false),
   m_paused(false),
   m_lastOdomTime(0.0),
   m_mustUpdateOffset(false), m_initializeFromIMU(false),
   m_initializeFromOdometry(false), m_isInitialized(false)
{
    parse_command_line(argc,argv);
    if (   ! connectNaoQi() || !  connectProxy() )
    {
      ROS_ERROR("Gosh! Throwsing exception");
      throw std::exception();
    }
    m_dataNamesList = AL::ALValue::array("DCM/Time",
         "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value","Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
         "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value",
         "Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value", "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value", 
         "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
         "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value", "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
         "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");


    // get update frequency. default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
    m_privateNh.param("sensor_rate", m_rate,m_rate);

    // get base_frame_id (and fix prefix if necessary)
    m_privateNh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
    m_privateNh.param("odom_frame_id", m_odomFrameId, m_odomFrameId);
    m_privateNh.param("use_imu_angles", m_useIMUAngles, m_useIMUAngles);

    m_pauseOdomSrv = m_nh.advertiseService("pause_odometry", &NaoSensors::pauseOdomCallback, this);
    m_resumeOdomSrv = m_nh.advertiseService("resume_odometry", &NaoSensors::resumeOdomCallback, this);
    m_odomOffsetSrv = m_nh.advertiseService("odometry_offset", &NaoSensors::odomOffsetCallback, this);
    m_setOdomPoseSrv = m_nh.advertiseService("set_odometry_pose", &NaoSensors::setOdomPoseCallback, this);


    m_odom.header.frame_id = m_odomFrameId;
    m_torsoIMU.header.frame_id = m_baseFrameId;
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
    std::copy(m_jointState.name.begin(), m_jointState.name.end()-1, std::ostream_iterator<std::string>(ss,","));
    std::copy(m_jointState.name.end()-1, m_jointState.name.end(), std::ostream_iterator<std::string>(ss));
    ROS_INFO("Nao joints found: %s",ss.str().c_str());

    if (m_useIMUAngles)
        ROS_INFO("Using IMU for odometry roll & pitch");


    // default values of transforms:
    m_odomTransformMsg.header.frame_id = m_odomFrameId;
    m_odomTransformMsg.child_frame_id = m_baseFrameId;

    m_odomOffset = tf::Transform(tf::createIdentityQuaternion());
    m_odomPose = tf::Transform(tf::createIdentityQuaternion());

    m_odomPub = m_nh.advertise<nav_msgs::Odometry>("odom",5);
    m_torsoIMUPub = m_nh.advertise<sensor_msgs::Imu>("imu",5);
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
   float odomX, odomY, odomZ, odomWX, odomWY, odomWZ;
   
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

      /******************************************************************
       *                              IMU
       *****************************************************************/
      if (memData.size() != m_dataNamesList.getSize())
      {
         ROS_ERROR("memData length %zu does not match expected length %u",memData.size(),m_dataNamesList.getSize() );
         continue;
      }
      // IMU data:
      m_torsoIMU.header.stamp = stamp;

      float angleX = memData[1];
      float angleY = memData[2];
      float angleZ = memData[3];
      float gyroX = memData[4];
      float gyroY = memData[5];
      float gyroZ = memData[6];
      float accX = memData[7];
      float accY = memData[8];
      float accZ = memData[9];

      m_torsoIMU.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                angleX,
                                angleY,
                                angleZ); // yaw currently always 0

      m_torsoIMU.angular_velocity.x = gyroX;
      m_torsoIMU.angular_velocity.y = gyroY;
      m_torsoIMU.angular_velocity.z = gyroZ; // currently always 0

      m_torsoIMU.linear_acceleration.x = accX;
      m_torsoIMU.linear_acceleration.y = accY;
      m_torsoIMU.linear_acceleration.z = accZ;

      // covariances unknown
      // cf http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html
      m_torsoIMU.orientation_covariance[0] = -1;
      m_torsoIMU.angular_velocity_covariance[0] = -1;
      m_torsoIMU.linear_acceleration_covariance[0] = -1;

      m_torsoIMUPub.publish(m_torsoIMU);


      /******************************************************************
       *                            Joint state
       *****************************************************************/
      m_jointState.header.stamp = stamp;
      m_jointState.header.frame_id = m_baseFrameId;
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


        /******************************************************************
        *                            Odometry
        *****************************************************************/
        if (!m_paused) {

        // apply offset transformation:
        tf::Pose transformedPose;


        if (odomData.size()!=6)
        {
            ROS_ERROR( "Error getting odom data. length is %zu",odomData.size() );
            continue;
        }

        double dt = (stamp.toSec() - m_lastOdomTime);

        odomX = odomData[0];
        odomY = odomData[1];
        odomZ = odomData[2];
        odomWX = odomData[3];
        odomWY = odomData[4];
        odomWZ = odomData[5];

        tf::Quaternion q;
        // roll and pitch from IMU, yaw from odometry:
        if (m_useIMUAngles)
            q.setRPY(angleX, angleY, odomWZ);
        else
            q.setRPY(odomWX, odomWY, odomWZ);

        m_odomPose.setOrigin(tf::Vector3(odomX, odomY, odomZ));
        m_odomPose.setRotation(q);

        if(m_mustUpdateOffset) {
            if(!m_isInitialized) {
                if(m_initializeFromIMU) {
                    // Initialization from IMU: Take x, y, z, yaw from odometry, roll and pitch from IMU
                    m_targetPose.setOrigin(m_odomPose.getOrigin());
                    m_targetPose.setRotation(tf::createQuaternionFromRPY(angleX, angleY, odomWZ));
                } else if(m_initializeFromOdometry) {
                    m_targetPose.setOrigin(m_odomPose.getOrigin());
                    m_targetPose.setRotation(tf::createQuaternionFromRPY(odomWX, odomWY, odomWZ));
                }
                m_isInitialized = true;
            } else {
                // Overwrite target pitch and roll angles with IMU data
                const double target_yaw = tf::getYaw(m_targetPose.getRotation());
                if(m_initializeFromIMU) {
                    m_targetPose.setRotation(tf::createQuaternionFromRPY(angleX, angleY, target_yaw));
                } else if(m_initializeFromOdometry){
                    m_targetPose.setRotation(tf::createQuaternionFromRPY(odomWX, odomWY, target_yaw));
                }
            }
            m_odomOffset = m_targetPose * m_odomPose.inverse();
            transformedPose = m_targetPose;
            m_mustUpdateOffset = false;
        } else {
            transformedPose = m_odomOffset * m_odomPose;
        }

        //
        // publish the transform over tf first
        //
        m_odomTransformMsg.header.stamp = stamp;
        tf::transformTFToMsg(transformedPose, m_odomTransformMsg.transform);
        m_transformBroadcaster.sendTransform(m_odomTransformMsg);


        //
        // Fill the Odometry msg
        //
        m_odom.header.stamp = stamp;
        //set the velocity first (old values still valid)
        m_odom.twist.twist.linear.x = (odomX - m_odom.pose.pose.position.x) / dt;
        m_odom.twist.twist.linear.y = (odomY - m_odom.pose.pose.position.y) / dt;
        m_odom.twist.twist.linear.z = (odomZ - m_odom.pose.pose.position.z) / dt;
        
        // TODO: calc angular velocity!
        //	m_odom.twist.twist.angular.z = vth; ??

        //set the position from the above calculated transform
        m_odom.pose.pose.orientation = m_odomTransformMsg.transform.rotation;
        m_odom.pose.pose.position.x = m_odomTransformMsg.transform.translation.x;
        m_odom.pose.pose.position.y = m_odomTransformMsg.transform.translation.y;
        m_odom.pose.pose.position.z = m_odomTransformMsg.transform.translation.z;


        m_odomPub.publish(m_odom);


        m_lastOdomTime = stamp.toSec();

        }
    }
    ROS_INFO("nao_sensors stopped.");

}

bool NaoSensors::pauseOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    if (m_paused){
        ROS_WARN("Odometry pause requested, but is already paused");
        return false;
    } else{
        ROS_INFO("Odometry paused");
        m_paused = true;
        m_targetPose = m_odomOffset * m_odomPose;
        m_mustUpdateOffset = true;
        return true;
    }
}

bool NaoSensors::resumeOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    if (m_paused){
        ROS_INFO("Odometry resumed");
        m_paused = false;
        return true;
    } else{
        ROS_WARN("Odometry resume requested, but is not paused");
        return false;
    }
}

bool NaoSensors::odomOffsetCallback(nao_msgs::SetTransform::Request& req, nao_msgs::SetTransform::Response& res){
    ROS_INFO("New odometry offset received");
    tf::Transform newOffset;
    tf::transformMsgToTF(req.offset, newOffset);

    // add new offset to current (transformed) odom pose:
    if(!m_mustUpdateOffset) {
        m_mustUpdateOffset = true;
        m_targetPose = m_odomOffset * m_odomPose * newOffset;
    } else {
        m_targetPose = m_targetPose * newOffset;
    }

    return true;
}

bool NaoSensors::setOdomPoseCallback(nao_msgs::SetTransform::Request& req, nao_msgs::SetTransform::Response& res){
    ROS_INFO("New target for current odometry pose received");
    tf::Transform targetPose;
    tf::transformMsgToTF(req.offset, targetPose);

    m_odomOffset = targetPose * m_odomPose.inverse();

    return true;
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
   
   sensors->run();
   
   delete sensors;

   ROS_INFO("nao_sensors stopped");
   return 0;
}
