cmake_minimum_required(VERSION 2.8.3)
project(nao_driver)

# We need to find catkin for the rest of the script to work
find_package(catkin REQUIRED)

# This will install python libraries
catkin_python_setup()

# At the moment, there's no C++ information to export
catkin_package()

#And this will install python nodes
install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )


# TODO Catkinize code to compile nao_sensors cpp binary (see old code below)

# Crosscompile nao_sensors to run as CPP binary on the robot (more efficient)
# Not needed for remote connection (use Python nodes).
#IF(OE_CROSS_BUILD)
#  include_directories(
#    $ENV{AL_DIR}/include
#    $ENV{AL_DIR}/include/alcommon
#    $ENV{AL_DIR}/include/alvision
#    $ENV{AL_DIR}/include/alcore
#    $ENV{AL_DIR}/include/alproxies
#    $ENV{AL_DIR}/include/alvalue
#    $ENV{AL_DIR}/include/altools
#    $ENV{AL_DIR}/include/alfactory
#    $ENV{AL_DIR}/include/libthread
#    $ENV{AL_DIR}/include/alvision
#  )

#  link_directories($ENV{AL_DIR}/lib)

  #if ( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  # Host is 64 bit machine, Aldebaran SDK is 32 bit
  # --> force 32 bit compilation mode by setting compiler switch
  #	set(CMAKE_C_FLAGS -m32)
  #	set(CMAKE_CXX_FLAGS -m32)
  #endif ( CMAKE_SIZEOF_VOID_P EQUAL 8 )

#  rosbuild_add_executable(nao_sensors_cpp src/nao_sensors_cpp.cpp)
#  target_link_libraries(nao_sensors_cpp alcommon alvision alvalue )
  
#ENDIF(OE_CROSS_BUILD)
