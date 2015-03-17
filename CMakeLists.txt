cmake_minimum_required(VERSION 2.8.3)
project(topic_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  INCLUDE_DIRS include/topic_relay
  CATKIN_DEPENDS roscpp roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/topic_relay
)

add_executable(relay src/topic_relay.cpp)

target_link_libraries(relay
  ${catkin_LIBRARIES}
)
