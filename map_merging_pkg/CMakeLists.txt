cmake_minimum_required(VERSION 2.8.3)
project(map_merging_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)
find_package(OpenCV)




## Generate messages in the 'msg' folder
## add_message_files(
##   FILES
##   Num.msg
## )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   #std_msgs
 )
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

## INCLUYENDO EJECUTABLES
add_executable(map_merging_pkg src/map_merging.cpp)
target_link_libraries(map_merging_pkg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(map_merging_pkg map_merging_pkg_generate_messages_cpp)



