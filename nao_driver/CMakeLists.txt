cmake_minimum_required(VERSION 2.8.3)
project(nao_driver)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# We need to find catkin for the rest of the script to work
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nao_msgs)
find_package(NAOqi QUIET COMPONENTS alcommon alvision alerror alvalue )
find_package(Boost QUIET COMPONENTS program_options)

# This will install python libraries
catkin_python_setup()

# At the moment, there's no C++ information to export
catkin_package()


if( ${NAOqi_FOUND} AND ${Boost_FOUND} )
    message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

    add_executable(nao_sensors_cpp src/nao_sensors_cpp.cpp)
    target_link_libraries(nao_sensors_cpp
        ${catkin_LIBRARIES}
        ${NAOqi_LIBRARIES}
        ${Boost_LIBRARIES})
    add_dependencies(nao_sensors_cpp ${catkin_EXPORTED_TARGETS})

    install(TARGETS nao_sensors_cpp
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
else()
    message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()

#And this will install python nodes
install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
