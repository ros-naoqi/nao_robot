cmake_minimum_required(VERSION 2.8.3)
project(nao_apps)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
)

catkin_install_python(PROGRAMS ./nodes/nao_alife.py ./nodes/nao_behaviors.py ./nodes/nao_diagnostic_updater.py
    ./nodes/nao_footsteps.py ./nodes/nao_leds.py ./nodes/nao_speech.py ./nodes/nao_tactile.py ./nodes/nao_walker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
