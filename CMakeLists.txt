cmake_minimum_required(VERSION 2.8.3)
project(position_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  message_generation
  visualization_msgs
  pcl_ros
)

add_message_files(
  FILES
  Anchor_msgs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pos_est src/position_estimation.cpp)
target_link_libraries(pos_est ${catkin_LIBRARIES})

#add_executable(pos_est_sim src/position_estimation_prototype_simulation.cpp)
#target_link_libraries(pos_est_sim ${catkin_LIBRARIES})

add_executable(anchor_0_2tag_listener src/anchor_0_2tag_listener.cpp)
target_link_libraries(anchor_0_2tag_listener ${catkin_LIBRARIES})

#add_executable(anchor_0_listener src/anchor_0_listener.cpp)
#target_link_libraries(anchor_0_listener ${catkin_LIBRARIES})

#add_executable(tag_listener src/tag_listener.cpp)
#target_link_libraries(tag_listener ${catkin_LIBRARIES})

#add_executable(tag_listener_async src/tag_listener_async.cpp)
#target_link_libraries(tag_listener_async ${catkin_LIBRARIES})
