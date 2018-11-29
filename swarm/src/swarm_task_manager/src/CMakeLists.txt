cmake_minimum_required(VERSION 2.8.3)
project(swarm_task_manager)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  message_generation
)
find_package(swarm_msgs REQUIRED COMPONENTS
    std_msgs
    message_generation)
find_package(swarm_core REQUIRED)

find_package(Boost 1.49.0 REQUIRED)
#find_package(Eigen3 REQUIRED)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

add_service_files(
    FILES
    resync.srv
    )

generate_messages(
    DEPENDENCIES std_msgs swarm_msgs
    )


catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS message_runtime roscpp std_msgs swarm_msgs swarm_core
  INCLUDE_DIRS ../include
  #  LIBRARIES swarm_msgs agent swarm_tools ros_swarm_tools swarm_core
)

include_directories(../include
    ${catkin_INCLUDE_DIRS}
    ${swarm_msgs_INCLUDE_DIRS}
    ${swarm_core_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


