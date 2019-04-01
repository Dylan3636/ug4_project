cmake_minimum_required(VERSION 2.8.3)
project(swarm_control)

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
# find_package(swarm_task_manager REQUIRED COMPONENTS
#     swarm_msgs
#     std_msgs
#     message_generation)
find_package(Boost 1.49.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

#add_service_files(
#    FILES
#    CollisionAvoidance.srv
#    )

generate_messages(
    DEPENDENCIES std_msgs swarm_msgs
    )

catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS message_runtime roscpp std_msgs swarm_msgs swarm_core
  LIBRARIES collision_avoidance motion_goal_control agent_control
  INCLUDE_DIRS ../include
)

include_directories(../include
    ${catkin_INCLUDE_DIRS}
    ${swarm_msgs_INCLUDE_DIRS}
    ${swarm_core_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS})

add_library(collision_avoidance collision_avoidance.cpp)
target_link_libraries(collision_avoidance ${swarm_core_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  collision_avoidance
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(motion_goal_control motion_goal_control.cpp)
target_link_libraries(motion_goal_control ${swarm_core_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  motion_goal_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(agent_control agent_control.cpp)
target_link_libraries(agent_control ${swarm_core_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  agent_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#add_executable(collision_avoidance_service collision_avoidance_service.cpp)
#add_dependencies(collision_avoidance_service collision_avoidance swarm_control_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(collision_avoidance_service collision_avoidance ${swarm_core_LIBRARIES} ${catkin_LIBRARIES})

add_executable(intruder_planner intruder_planner.cpp)
add_dependencies(intruder_planner collision_avoidance motion_goal_control swarm_control_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(intruder_planner motion_goal_control collision_avoidance ${swarm_core_LIBRARIES} ${catkin_LIBRARIES})

