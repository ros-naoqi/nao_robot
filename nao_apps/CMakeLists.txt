cmake_minimum_required(VERSION 2.8.3)
project(nao_apps)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
)

install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
