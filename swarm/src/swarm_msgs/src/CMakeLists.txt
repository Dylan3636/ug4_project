cmake_minimum_required(VERSION 2.8.3)
project(swarm_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
)

# find_package(Boost REQUIRED thread)

add_message_files(
  DIRECTORY msg
  FILES
  agentState.msg
  agentCommand.msg
  worldState.msg
  agentConstraints.msg
  agentParam.msg
  agentType.msg
  agentAssignment.msg
  usvAgent.msg
  intruderAgent.msg
  simulationMarker.msg
  taskType.msg
  agentTask.msg
  swarmAssignment.msg
  threatStatistics.msg
  resetSystem.msg
  initializeSystem.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs
    )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
