cmake_minimum_required(VERSION 2.8.3)
project(swarm_threat_detection)

find_package(catkin REQUIRED COMPONENTS
  swarm_msgs
  cmake_modules
  std_msgs
  roslaunch
  message_generation
)


set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

roslaunch_add_file_check(launch)

add_service_files(
        FILES
        ThreatDetection.srv
        batchIntruderCommands.srv
)

generate_messages(
    DEPENDENCIES std_msgs swarm_msgs
    )

catkin_package(
  CATKIN_DEPENDS std_msgs swarm_msgs
)

catkin_install_python(PROGRAMS scripts/basic_threat_detection.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
