cmake_minimum_required(VERSION 3.0.2)
project(camera_node)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_package()

add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)
