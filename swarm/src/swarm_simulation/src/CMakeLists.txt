cmake_minimum_required(VERSION 2.8.3)
project(swarm_simulation)

find_package(catkin REQUIRED COMPONENTS
  swarm_msgs
  cmake_modules
  std_msgs
)
find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

#set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

catkin_package(
  CATKIN_DEPENDS std_msgs swarm_msgs
)

catkin_install_python(PROGRAMS scripts/sim.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
