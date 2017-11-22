cmake_minimum_required(VERSION 2.8.3)
project(tom_joint_planner)

# add c++11 support
if(CMAKE_COMPILER_IS_GNUCC)
    ADD_DEFINITIONS ( -std=c++11 )
endif(CMAKE_COMPILER_IS_GNUCC)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
)


## Declare a cpp executable
add_executable(TomJointPlannerSimulation src/TomJointPlannerSimulation.cpp include/tom_joint_planner/TomJointPlannerSimulation.h)



## Specify libraries to link a library or executable target against
target_link_libraries(TomJointPlannerSimulation ${catkin_LIBRARIES})






