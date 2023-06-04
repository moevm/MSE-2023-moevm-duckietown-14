cmake_minimum_required(VERSION 2.8.3)
project(draw-line-crossroad)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()
