cmake_minimum_required(VERSION 2.8.3)
project(swarm_core)

find_package(catkin REQUIRED COMPONENTS
  swarm_msgs
  cmake_modules
  roscpp
  std_msgs
)

find_package(Boost 1.49.0 REQUIRED)
#find_package(Eigen3 REQUIRED)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS roscpp std_msgs swarm_msgs
  INCLUDE_DIRS ../include
  LIBRARIES agent usv_swarm swarm_tools ros_swarm_tools
)

include_directories(../include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen3_INCLUDE_DIRS})

#install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml"
#)

add_library(agent agent.cpp)
add_dependencies(agent swarm_tools)
target_link_libraries(agent ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  agent
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_library(usv_swarm usv_swarm.cpp)
target_link_libraries(usv_swarm agent ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  usv_swarm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

add_library(swarm_tools swarm_tools.cpp)
target_link_libraries(swarm_tools ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  swarm_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(ros_swarm_tools ros_swarm_tools.cpp)
target_link_libraries(ros_swarm_tools usv_swarm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS
  ros_swarm_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#install(DIRECTORY cfg/cpp/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#)

# add_executable(local_planner local_planner.cpp)
# add_dependencies(local_planner agent swarm_tools collision_avoidance swarm_msgs_generate_msgs_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(local_planner agent swarm_tools collision_avoidance ${catkin_LIBRARIES})

#install(FILES nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
