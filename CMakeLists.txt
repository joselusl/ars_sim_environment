cmake_minimum_required(VERSION 3.0.0)
project(ars_sim_environment)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
  DEPENDS
  CATKIN_DEPENDS rospy std_msgs geometry_msgs visualization_msgs python-rospkg
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
