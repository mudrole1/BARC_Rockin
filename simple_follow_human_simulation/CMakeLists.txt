cmake_minimum_required(VERSION 2.8.3)
project(follow_sim)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  laser_geometry
  sensor_msgs
  joy
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files( DIRECTORY msg
   FILES
   Pose.msg
   Recognizer.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs sensor_msgs )

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
)

###########
## Build ##
###########

# include_directories(include)
include_directories( include  ${catkin_INCLUDE_DIRS} )


add_executable(Recognizer src/Recognizer01.cpp)
target_link_libraries( Recognizer ${catkin_LIBRARIES} )
add_dependencies( Recognizer follow_sim_generate_messages_cpp )

add_executable(Tracker src/Tracker01.cpp)
target_link_libraries( Tracker ${catkin_LIBRARIES} )
add_dependencies( Tracker follow_sim_generate_messages_cpp )

add_executable(Navigator src/Navigator01.cpp)
target_link_libraries( Navigator ${catkin_LIBRARIES} )
add_dependencies( Tracker follow_sim_generate_messages_cpp )

add_executable(JoyToRobot src/JoyToRobot.cpp)
target_link_libraries( JoyToRobot ${catkin_LIBRARIES} )
add_dependencies( Tracker follow_sim_generate_messages_cpp )

