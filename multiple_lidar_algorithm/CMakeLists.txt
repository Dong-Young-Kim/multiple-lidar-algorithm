cmake_minimum_required(VERSION 3.0.2)
project(multiple_lidar_algorithm)


find_package(catkin REQUIRED
    roscpp
    pcl_conversions
    pcl_ros
    std_msgs
    message_generation
)

find_package( OpenCV REQUIRED )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(multiple_lidar_algorithm_node src/mutiple_lidar_node.cpp)
target_link_libraries(multiple_lidar_algorithm_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )