cmake_minimum_required(VERSION 2.8.3)
project(nao_pose)

# Find all required modules for compilation
find_package(catkin REQUIRED)

# This will install python libraries
catkin_python_setup()

# Declare info that other packages need to import messages generated here
catkin_package( )

#And this will install python nodes
install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)


#And this will install python nodes
install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Install config files
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
