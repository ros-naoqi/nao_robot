#!/usr/bin/env python

#
# ROS node to provide joint angle control to Nao by wrapping NaoQI
# This code is currently compatible to NaoQI version 1.6 or newer (latest 
# tested: 1.12)
#
# Copyright 2011 Armin Hornung, University of Freiburg
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
import actionlib
from nao_msgs.msg import(
    JointTrajectoryResult,
    JointTrajectoryAction,
    JointAnglesWithSpeed,
    JointAnglesWithSpeedResult,
    JointAnglesWithSpeedAction,
    BodyPoseWithSpeedAction,
    BodyPoseWithSpeedGoal,
    BodyPoseWithSpeedResult)
 
from nao_driver import NaoNode

from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState

class NaoController(NaoNode):
    def __init__(self): 
        NaoNode.__init__(self, 'nao_controller')
        
        self.connectNaoQi()
        
        # store the number of joints in each motion chain and collection, used for sanity checks
        self.collectionSize = {}
        for collectionName in ['Head', 'LArm', 'LLeg', 'RLeg', 'RArm', 'Body', 'BodyJoints', 'BodyActuators']:
            try:
                self.collectionSize[collectionName] = len(self.motionProxy.getJointNames(collectionName));
            except RuntimeError:
                # the following is useful for old NAOs with no legs/arms
                rospy.logwarn('Collection %s not found on your robot.' % collectionName)

        # Get poll rate for actionlib (ie. how often to check whether currently running task has been preempted)
        # Defaults to 200ms
        self.poll_rate = int(rospy.get_param("~poll_rate", 0.2)*1000)
        
        # initial stiffness (defaults to 0 so it doesn't strain the robot when no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)
        
        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)


        # start services / actions:
        self.enableStiffnessSrv = rospy.Service("body_stiffness/enable", Empty, self.handleStiffnessSrv)
        self.disableStiffnessSrv = rospy.Service("body_stiffness/disable", Empty, self.handleStiffnessOffSrv)


        #Start simple action servers
        self.jointTrajectoryServer = actionlib.SimpleActionServer("joint_trajectory", JointTrajectoryAction, 
                                                                  execute_cb=self.executeJointTrajectoryAction,
                                                                  auto_start=False)
        
        self.jointStiffnessServer = actionlib.SimpleActionServer("joint_stiffness_trajectory", JointTrajectoryAction, 
                                                                  execute_cb=self.executeJointStiffnessAction,
                                                                  auto_start=False)
        
        self.jointAnglesServer = actionlib.SimpleActionServer("joint_angles_action", JointAnglesWithSpeedAction, 
                                                                  execute_cb=self.executeJointAnglesWithSpeedAction,
                                                                  auto_start=False)

        #Start action servers
        self.jointTrajectoryServer.start()
        self.jointStiffnessServer.start()
        self.jointAnglesServer.start()

        # only start when ALRobotPosture proxy is available
        if not (self.robotPostureProxy is None):
            self.bodyPoseWithSpeedServer = actionlib.SimpleActionServer("body_pose_naoqi", BodyPoseWithSpeedAction,
                                              execute_cb=self.executeBodyPoseWithSpeed,
                                              auto_start=False)
            self.bodyPoseWithSpeedServer.start()
        else:
            rospy.logwarn("Proxy to ALRobotPosture not available, requests to body_pose_naoqi will be ignored.")

        # subscribers last:
        rospy.Subscriber("joint_angles", JointAnglesWithSpeed, self.handleJointAngles, queue_size=10)
        rospy.Subscriber("joint_stiffness", JointState, self.handleJointStiffness, queue_size=10)

        rospy.loginfo("nao_controller initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

        # optional, newly introduced in 1.14
        self.robotPostureProxy = self.get_proxy("ALRobotPosture")
            
            
    def handleJointAngles(self, msg):
        rospy.logdebug("Received a joint angle target")
        try:
            # Note: changeAngles() and setAngles() are non-blocking functions.
            if (msg.relative==0):
                self.motionProxy.setAngles(list(msg.joint_names), list(msg.joint_angles), msg.speed)
            else:
                self.motionProxy.changeAngles(list(msg.joint_names), list(msg.joint_angles), msg.speed)
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            
    def handleJointStiffness(self, msg):
        rospy.logdebug("Received a joint angle stiffness")
        try:
            self.motionProxy.setStiffnesses(list(msg.name), list(msg.effort))
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)

    def handleStiffnessSrv(self, req):
        try:
            self.motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
            rospy.loginfo("Body stiffness enabled")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleStiffnessOffSrv(self, req):
        try:
            self.motionProxy.stiffnessInterpolation("Body", 0.0, 0.5)
            rospy.loginfo("Body stiffness removed")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            

    def jointTrajectoryGoalMsgToAL(self, goal):
        """Helper, convert action goal msg to Aldebraran-style arrays for NaoQI"""
        names = list(goal.trajectory.joint_names)
#        if goal.trajectory.joint_names == ["Body"]:
#            names = self.motionProxy.getJointNames('Body')
       
        if len(goal.trajectory.points) == 1 and len(goal.trajectory.points[0].positions) == 1:
            angles = goal.trajectory.points[0].positions[0]
        else:
            angles = list(list(p.positions[i] for p in goal.trajectory.points) for i in range(0,len(goal.trajectory.points[0].positions)))
        
        #strip 6,7 and last 2 from angles if the pose was for H25 but we're running an H21
        if not isinstance(angles, float) and len(angles) > self.collectionSize["Body"]:
            rospy.loginfo("Stripping angles from %d down to %d", len(angles), self.collectionSize["Body"])
            angles.pop(6)
            angles.pop(7)
            angles.pop()
            angles.pop()
            
        if len(names) > self.collectionSize["Body"]:
            rospy.loginfo("Stripping names from %d down to %d", len(names), self.collectionSize["Body"])
            names.pop(6)
            names.pop(7)
            names.pop()
            names.pop()
        
        times = list(p.time_from_start.to_sec() for p in goal.trajectory.points)
        if len(times) == 1 and len(names) == 1:
            times = times[0]
        if (len(names) > 1):
            times = [times]*len(names)
            
        return (names, angles, times)
    

    def executeJointTrajectoryAction(self, goal):
        rospy.loginfo("JointTrajectory action executing");
                    
        names, angles, times = self.jointTrajectoryGoalMsgToAL(goal)
                
        rospy.logdebug("Received trajectory for joints: %s times: %s", str(names), str(times))     
        rospy.logdebug("Trajectory angles: %s", str(angles))

        task_id = None
        running = True
        #Wait for task to complete
        while running and not self.jointTrajectoryServer.is_preempt_requested() and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                # ...Start it in another thread (thanks to motionProxy.post)
                task_id = self.motionProxy.post.angleInterpolation(names, angles, times, (goal.relative==0))
            
            #Wait for a bit to complete, otherwise check we can keep running
            running = self.motionProxy.wait(task_id, self.poll_rate)
        
        # If still running at this point, stop the task
        if running and task_id:
            self.motionProxy.stop( task_id )
        
        jointTrajectoryResult = JointTrajectoryResult()
        jointTrajectoryResult.goal_position.header.stamp = rospy.Time.now()
        jointTrajectoryResult.goal_position.position = self.motionProxy.getAngles(names, True)
        jointTrajectoryResult.goal_position.name = names

        if not self.checkJointsLen(jointTrajectoryResult.goal_position):
            self.jointTrajectoryServer.set_aborted(jointTrajectoryResult)
            rospy.logerr("JointTrajectory action error in result: sizes mismatch")
        
        elif running:
            self.jointTrajectoryServer.set_preempted(jointTrajectoryResult)
            rospy.logdebug("JointTrajectory preempted")
        
        else:
            self.jointTrajectoryServer.set_succeeded(jointTrajectoryResult)
            rospy.loginfo("JointTrajectory action done")


    def executeJointStiffnessAction(self, goal):
        rospy.loginfo("JointStiffness action executing");
                    
        names, angles, times = self.jointTrajectoryGoalMsgToAL(goal)
        
        rospy.logdebug("Received stiffness trajectory for joints: %s times: %s", str(names), str(times))     
        rospy.logdebug("stiffness values: %s", str(angles))

        task_id = None
        running = True
        #Wait for task to complete
        while running and not self.jointStiffnessServer.is_preempt_requested() and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                # ...Start it in another thread (thanks to motionProxy.post)
                task_id = self.motionProxy.post.stiffnessInterpolation(names, angles, times)
            
            #Wait for a bit to complete, otherwise check we can keep running
            running = self.motionProxy.wait(task_id, self.poll_rate)

        # If still running at this point, stop the task
        if running and task_id:
            self.motionProxy.stop( task_id )
            
        jointStiffnessResult = JointTrajectoryResult()
        jointStiffnessResult.goal_position.header.stamp = rospy.Time.now()
        jointStiffnessResult.goal_position.position = self.motionProxy.getStiffnesses(names)
        jointStiffnessResult.goal_position.name = names

        if not self.checkJointsLen(jointStiffnessResult.goal_position):
            self.jointStiffnessServer.set_aborted(jointStiffnessResult)
            rospy.logerr("JointStiffness action error in result: sizes mismatch")
        
        elif running:
            self.jointStiffnessServer.set_preempted(jointStiffnessResult)
            rospy.logdebug("JointStiffness preempted")
        
        else:
            self.jointStiffnessServer.set_succeeded(jointStiffnessResult)
            rospy.loginfo("JointStiffness action done")
            

    def executeJointAnglesWithSpeedAction(self, goal):           
        
        names = list(goal.joint_angles.joint_names)
        angles = list(goal.joint_angles.joint_angles)
        rospy.logdebug("Received JointAnglesWithSpeed for joints: %s angles: %s", str(names), str(angles))
            
        if goal.joint_angles.relative == 1:
            # TODO: this uses the current angles instead of the angles at the given timestamp
            currentAngles = self.motionProxy.getAngles(names, True)
            angles = list(map(lambda x,y: x+y, angles, currentAngles))            

        task_id = None
        running = True
        #Wait for task to complete
        while running and not self.jointAnglesServer.is_preempt_requested() and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                # ...Start it in another thread (thanks to motionProxy.post)
                task_id = self.motionProxy.post.angleInterpolationWithSpeed(names, angles, goal.joint_angles.speed)
            
            #Wait for a bit to complete, otherwise check we can keep running
            running = self.motionProxy.wait(task_id, self.poll_rate)
        
        # If still running at this point, stop the task
        if running and task_id:
            self.motionProxy.stop( task_id )
            
        jointAnglesWithSpeedResult = JointAnglesWithSpeedResult()
        jointAnglesWithSpeedResult.goal_position.header.stamp = rospy.Time.now()
        jointAnglesWithSpeedResult.goal_position.position = self.motionProxy.getAngles(names, True)
        jointAnglesWithSpeedResult.goal_position.name = names

        if not self.checkJointsLen(jointAnglesWithSpeedResult.goal_position):
            self.jointAnglesServer.set_aborted(jointAnglesWithSpeedResult)
            rospy.logerr("JointAnglesWithSpeed action error in result: sizes mismatch")

        elif running:
            self.jointAnglesServer.set_preempted(jointAnglesWithSpeedResult)
            rospy.logdebug("JointAnglesWithSpeed preempted")

        else:
            self.jointAnglesServer.set_succeeded(jointAnglesWithSpeedResult)
            rospy.loginfo("JointAnglesWithSpeed action done")

    def checkJointsLen(self, goal_position):      
        if len(goal_position.name) == 1 and self.collectionSize.has_key(goal_position.name[0]):
            return len(goal_position.position) == self.collectionSize[goal_position.name[0]] 
        else:
            return len(goal_position.position) ==  len(goal_position.name)
            
    def executeBodyPoseWithSpeed(self, goal):
      
        #~ Sanity checks
        if (goal.speed < 0.0) or (goal.speed > 1.0):
            bodyPoseWithSpeedResult = BodyPoseWithSpeedResult()
            self.bodyPoseWithSpeedServer.set_aborted(bodyPoseWithSpeedResult)
            rospy.logerr("Body pose setter: Not a valid speed value.")
            return
      
        valid_postures = self.robotPostureProxy.getPostureList()

        if goal.posture_name not in valid_postures:
            bodyPoseWithSpeedResult = BodyPoseWithSpeedResult()
            self.bodyPoseWithSpeedServer.set_aborted(bodyPoseWithSpeedResult)  
            rospy.logerr("Body pose setter: Not a valid posture.")
            return

        #~ Must set stiffness on
        try:
            self.motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
            rospy.loginfo("Body stiffness enabled")
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return
          
        #~ Go to posture. This is blocking
        self.robotPostureProxy.goToPosture(goal.posture_name, goal.speed)
        #~ Return success
        self.bodyPoseWithSpeedServer.set_succeeded()

if __name__ == '__main__':

    controller = NaoController()
    rospy.loginfo("nao_controller running...")
    rospy.spin()
    
    rospy.loginfo("nao_controller stopped.")
    exit(0)
