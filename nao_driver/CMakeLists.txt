cmake_minimum_required(VERSION 2.8.3)
project(nao_driver)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    sensor_msgs
    geometry_msgs
    nao_msgs
    dynamic_reconfigure
    diagnostic_aggregator
    diagnostic_msgs)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue
)

find_package(Boost QUIET COMPONENTS program_options signals)

# This will install python libraries
catkin_python_setup()

# Generate dynamic reconfigure files
generate_dynamic_reconfigure_options( config/nao_speech.cfg )

# Information for other packages to import this module
catkin_package( CATKIN_DEPENDS dynamic_reconfigure
   CFG_EXTRAS cmake_modules-extras.cmake
)

if( ${NAOqi_FOUND} AND ${Boost_FOUND} )
    message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS})

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

# add a diagnostic analyzer plugin
add_library(nao_diagnostic_analyzers src/NaoJointsAnalyzer.cpp)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(nao_diagnostic_analyzers ${catkin_LIBRARIES})
install(TARGETS nao_diagnostic_analyzers DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nao_analyzer_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#And this will install python nodes
install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Install config file (& ignore dynamic reconfigure files)
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN "*.cfg" EXCLUDE)

## Install all cmake files
install(FILES cmake/FindNAOqi.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)
