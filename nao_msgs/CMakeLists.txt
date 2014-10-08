cmake_minimum_required(VERSION 2.8.3)
project(nao_msgs)

#List to make rest of code more readable
set( MESSAGE_DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    trajectory_msgs)

#Declare build dependencies
find_package(catkin REQUIRED 
    COMPONENTS
    message_generation
    ${MESSAGE_DEPENDENCIES} )

#Add message files
add_message_files(DIRECTORY msg
    FILES
    AudioBuffer.msg
    Bumper.msg
    FadeRGB.msg
    JointAnglesWithSpeed.msg
    JointAngleTrajectory.msg
    TactileTouch.msg
    WordRecognized.msg
)

#Add service files
add_service_files(DIRECTORY srv
    FILES
    CmdPoseService.srv
    CmdVelService.srv
    GetInstalledBehaviors.srv
    GetTruepose.srv
    SetTransform.srv
    SetArmsEnabled.srv
)

#Add action files
add_action_files(DIRECTORY action
    FILES
    Blink.action
    BodyPose.action
    BodyPoseWithSpeed.action
    FollowPath.action
    JointAnglesWithSpeed.action
    JointTrajectory.action
    RunBehavior.action 
    SetSpeechVocabulary.action
    SpeechWithFeedback.action
)

#And now generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

#Declare package run-time dependencies
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})

