cmake_minimum_required(VERSION 2.8.3)
project(swarm_simulation)

find_package(catkin REQUIRED COMPONENTS
  swarm_msgs
  cmake_modules
  std_msgs
)

# find_package(tkinter REQUIRED)
# find_package(numpy REQUIRED)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS std_msgs swarm_msgs
  swarm_msgs
)

# include_directories(../include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen3_INCLUDE_DIRS})

#install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml"
#)

# install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

#install(DIRECTORY cfg/cpp/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#)

catkin_install_python(PROGRAMS scripts/sim.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#add_dependencies( agent swarm_tools collision_avoidance swarm_msgs_generate_msgs_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(local_planner agent swarm_tools collision_avoidance ${catkin_LIBRARIES})

#install(FILES nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
