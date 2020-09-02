
cmake_minimum_required(VERSION 2.8.3)
project(freecad_to_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

