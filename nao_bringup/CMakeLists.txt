cmake_minimum_required(VERSION 2.8.3)
project(nao_bringup)
find_package(catkin REQUIRED)

catkin_package()

# install

install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
