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
find_package(swarm_control REQUIRED)

find_package(Boost 1.49.0 REQUIRED)
#find_package(Eigen3 REQUIRED)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

add_service_files(
    FILES
    resync.srv
    modelPredictiveSimulation.srv
    )

generate_messages(
    DEPENDENCIES std_msgs swarm_msgs
    )


catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS message_runtime roscpp std_msgs swarm_msgs swarm_core swarm_control
  INCLUDE_DIRS ../include
  LIBRARIES swarm_task_allocation
)

include_directories(../include
    ${catkin_INCLUDE_DIRS}
    ${swarm_msgs_INCLUDE_DIRS}
    ${swarm_core_INCLUDE_DIRS}
    ${swarm_control_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS})

add_library(swarm_task_allocation task_allocation_model_predictive.cpp)
target_link_libraries(swarm_task_allocation ${swarm_core_LIBRARIES} ${swarm_control_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  swarm_task_allocation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(task_manager swarm_task_manager.cpp)
# add_dependencies(task_manager swarm_task_allocation swarm_task_manager_gencpp ${swarm_core_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(task_manager swarm_task_allocation ${swarm_core_LIBRARIES} ${catkin_LIBRARIES})




