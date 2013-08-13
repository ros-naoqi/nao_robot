cmake_minimum_required(VERSION 2.8.3)
project(nao_description)

# We need to find catkin for the rest of the script to work
find_package(catkin REQUIRED)

# Nothing that can be imported by other modules
catkin_package()

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Instructions to install urdf files
install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

