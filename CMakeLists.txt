
cmake_minimum_required(VERSION 2.8.3)
project(humanoid_17dof_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

