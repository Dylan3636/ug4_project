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
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs
    )
