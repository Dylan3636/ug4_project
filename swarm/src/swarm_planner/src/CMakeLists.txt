cmake_minimum_required(VERSION 2.8.3)
project(swarm_planner)

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
find_package(swarm_control REQUIRED)
find_package(swarm_task_manager REQUIRED COMPONENTS
    swarm_msgs
    std_msgs
    message_generation)
find_package(Boost 1.49.0 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS message_runtime roscpp std_msgs swarm_msgs swarm_core swarm_control swarm_task_manager
  # INCLUDE_DIRS ../include
)

include_directories(../include
    ${catkin_INCLUDE_DIRS}
    ${swarm_msgs_INCLUDE_DIRS}
    ${swarm_core_INCLUDE_DIRS}
    ${swarm_control_INCLUDE_DIRS}
    ${swarm_task_manager_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add_executable(intruder_planner intruder_planner.cpp)
# add_dependencies(intruder_planner collision_avoidance motion_goal_control collision_avoidance_service swarm_control_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(intruder_planner motion_goal_control collision_avoidance ${swarm_core_LIBRARIES} ${catkin_LIBRARIES})

add_executable(usv_planner usv_planner.cpp)
#add_dependencies(usv_planner ${swarm_control_LIBRARIES} ${swarm_task_manager_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(usv_planner ${swarm_core_LIBRARIES} ${swarm_control_LIBRARIES} ${swarm_task_manager_LIBRARIES} ${catkin_LIBRARIES})

